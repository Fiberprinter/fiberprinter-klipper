import stepper, chelper

import math

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
        gcode.register_mux_command("SYNC_EXTRUDER_MOTION", "EXTRUDER",
                                   self.name, self.cmd_SYNC_EXTRUDER_MOTION,
                                   desc=self.cmd_SYNC_EXTRUDER_MOTION_help)
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
    
    def sync_to_extruder(self, extruder_name):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        if not extruder_name:
            self.stepper.set_trapq(None)
            self.motion_queue = None
            return
        extruder = self.printer.lookup_object(extruder_name, None)

        self.stepper.set_position([extruder.last_position, 0., 0.])
        self.stepper.set_trapq(extruder.get_trapq())
        self.motion_queue = extruder_name
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
    cmd_SYNC_EXTRUDER_MOTION_help = "Set extruder stepper motion queue"
    def cmd_SYNC_EXTRUDER_MOTION(self, gcmd):
        ename = gcmd.get('MOTION_QUEUE')
        self.sync_to_extruder(ename)
        gcmd.respond_info("Extruder '%s' now syncing with '%s'"
                          % (self.name, ename))

def load_extruders(config):
    printer = config.get_printer()
    for i in range(99):
        section = 'fiberextruder_stepper'
        if i:
            section = 'fiberextruder_stepper%d' % (i,)
            
        if not config.has_section(section):
            break
        
        pe = FiberExtruderStepper(config.getsection(section), i)
        printer.add_object(section, pe)