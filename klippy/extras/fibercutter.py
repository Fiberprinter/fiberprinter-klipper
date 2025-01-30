SERVO_SIGNAL_PERIOD = 0.020
PIN_MIN_TIME = 0.100

ANGLE_SPEED = 0.005
MIN_WIDTH = 0.0005
MAX_WIDTH = 0.00251

class CutterServo:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.min_width = 0.0005
        self.max_width = 0.00251
        self.max_angle = 180.0

        self.angle_to_width = (self.max_width - self.min_width) / self.max_angle
        self.width_to_value = 1. / SERVO_SIGNAL_PERIOD
        self.last_value = self.last_value_time = 0.

        self.initial_angle = config.getfloat('initial_angle', 20.0, minval=0., maxval=360.)
        self.cutting_angle = config.getfloat('cutting_angle', 20.0, minval=0., maxval=360.)
        # Setup mcu_servo pin
        ppins = self.printer.lookup_object('pins')
        self.mcu_servo = ppins.setup_pin('pwm', config.get('cutting_pin'))
        self.mcu_servo.setup_max_duration(0.)
        self.mcu_servo.setup_cycle_time(SERVO_SIGNAL_PERIOD)
        self.mcu_servo.setup_start_value(self._get_pwm_from_angle(self.initial_angle), 0.)
        # Register commands
        gcode = self.printer.lookup_object('gcode')

        gcode.register_command("CUT", self.cmd_SET_SERVO)
        gcode.register_command("M751", self.cmd_SET_SERVO)

        gcode.register_command("CUT_STATUS", self.get_last_value)
        
        
    def get_last_value(self, gcmd):
        gcmd.respond_info(f"Current Angle: {self._get_angle_from_pwm(self.last_value)}")
     
    def get_status(self, eventtime):
        return {'value': self.last_value}

    def _set_pwm(self, print_time, value):
        if value == self.last_value:
            return
        print_time = max(print_time, self.last_value_time + PIN_MIN_TIME)
        self.mcu_servo.set_pwm(print_time, value)
        self.last_value = value
        self.last_value_time = print_time

    def _get_pwm_from_angle(self, angle):
        angle = max(0., min(self.max_angle, angle))
        width = self.min_width + angle * self.angle_to_width
        return width * self.width_to_value
    
    def _get_angle_from_pwm(self, pwm):
        width = pwm / self.width_to_value
        angle = (width - self.min_width) / self.angle_to_width
        angle = max(0., min(self.max_angle, angle))
        return angle

    cmd_SET_SERVO_help = "Set servo angle"
    
    def cmd_SET_SERVO(self, gcmd):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        angle = gcmd.get_float('ANGLE', None)

        if angle is not None:
            self._set_pwm(print_time, self._get_pwm_from_angle(angle))
        else:
            self._set_pwm(print_time, self._get_pwm_from_angle(self.initial_angle + self.cutting_angle))
            self._set_pwm(print_time + self.cutting_angle * ANGLE_SPEED, self._get_pwm_from_angle(self.initial_angle))
            
def load_config(config):
    return CutterServo(config)