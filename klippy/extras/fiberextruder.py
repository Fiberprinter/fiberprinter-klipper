import math, logging
from kinematics.extruder import ExtruderStepper
import chelper

import stepper, chelper

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

class FiberExtruderStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        
        # Setup stepper
        self.stepper = stepper.PrinterStepper(config)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.sk_extruder = ffi_main.gc(ffi_lib.extruder_stepper_alloc(),
                                       ffi_lib.extruder_stepper_free)
        self.stepper.set_stepper_kinematics(self.sk_extruder)
        self.motion_queue = None
        # Register commands
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        gcode = self.printer.lookup_object('gcode')

        gcode.register_mux_command("SET_FIBER_EXTRUDER_ROTATION_DISTANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_FIBER_ROTATION_DISTANCE,
                                   desc=self.cmd_SET_FIBER_ROTATION_DISTANCE_help)
    def _handle_connect(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_step_generator(self.stepper.generate_steps)
        # self._set_pressure_advance(self.config_pa, self.config_smooth_time)
    def get_status(self, eventtime):
        return {'pressure_advance': self.pressure_advance,
                'smooth_time': self.pressure_advance_smooth_time,
                'motion_queue': self.motion_queue}
    def find_past_position(self, print_time):
        mcu_pos = self.stepper.get_past_mcu_position(print_time)
        return self.stepper.mcu_to_commanded_position(mcu_pos)

    cmd_SET_FIBER_ROTATION_DISTANCE_help = "Set extruder rotation distance"
    def cmd_SET_FIBER_ROTATION_DISTANCE(self, gcmd):
        rotation_dist = gcmd.get_float('DISTANCE', None)
        if rotation_dist is not None:
            if not rotation_dist:
                raise gcmd.error("Rotation distance can not be zero")
            invert_dir, orig_invert_dir = self.stepper.get_dir_inverted()
            next_invert_dir = orig_invert_dir
            if rotation_dist < 0.:
                next_invert_dir = not orig_invert_dir
                rotation_dist = -rotation_dist
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.flush_step_generation()
            self.stepper.set_rotation_distance(rotation_dist)
            self.stepper.set_dir_inverted(next_invert_dir)
        else:
            rotation_dist, spr = self.stepper.get_rotation_distance()
        invert_dir, orig_invert_dir = self.stepper.get_dir_inverted()
        if invert_dir != orig_invert_dir:
            rotation_dist = -rotation_dist
        gcmd.respond_info("Extruder '%s' rotation distance set to %0.6f"
                          % (self.name, rotation_dist))

# Tracking for hotend heater, extrusion motion queue, and extruder stepper
class FiberExtruder:
    def __init__(self, config, extruder_num):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.last_position = 0.
        self.extruder_num = extruder_num
        
        # Setup extruder trapq (trapezoidal motion queue)
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        
        # Setup extruder stepper
        self.extruder_stepper = None
        if (config.get('step_pin', None) is not None
            or config.get('dir_pin', None) is not None
            or config.get('rotation_distance', None) is not None):
            self.extruder_stepper = ExtruderStepper(config)
            self.extruder_stepper.stepper.set_trapq(self.trapq)
            
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        
    def _handle_connect(self):
        if self.extruder_num == 0:
            extruder = self.printer.lookup_object('extruder')
        else:
            extruder = self.printer.lookup_object('extruder%d' % (self.extruder_num,))
        
        extruder.register_update_move_time_callback(self.update_move_time)
        extruder.register_check_move_callback(self.check_move)
        extruder.register_move_callback(self.move)
    
    def update_move_time(self, flush_time, clear_history_time):
        self.trapq_finalize_moves(self.trapq, flush_time, clear_history_time)

    def check_move(self, move):
        pass

    def move(self, print_time, move):
        # fiber extruder move
        if move.axes_d[4]:
            axis_r = move.axes_r[4]
            accel = move.accel * axis_r
            start_v = move.start_v * axis_r
            cruise_v = move.cruise_v * axis_r
                
            # Queue movement (x is extruder movement, y is pressure advance flag)
            self.fiber_trapq_append(self.trapq, print_time,
                                move.accel_t, move.cruise_t, move.decel_t,
                                move.start_pos[4], 0., 0.,
                                1., False, 0.,
                                start_v, cruise_v, accel)
            
            self.fiber_last_position = move.end_pos[4]

def load_config(config):
    printer = config.get_printer()
    
    for i in range(99):
        section = 'fiberextruder'
        if i:
            section = 'fiberextruder%d' % (i,)
        if not config.has_section(section):
            break
        pe = FiberExtruder(config.getsection(section), i)
        printer.add_object(section, pe)