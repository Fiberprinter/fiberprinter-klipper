import math, logging
from kinematics.extruder import ExtruderStepper
import chelper

import math

# Tracking for hotend heater, extrusion motion queue, and extruder stepper
class FiberExtruder:
    def __init__(self, config, extruder_num):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.last_position = 0.
        self.fiber_last_position = 0.
        
        # Setup hotend heater
        pheaters = self.printer.load_object(config, 'heaters')
        gcode_id = 'T%d' % (extruder_num,)
        self.heater = pheaters.setup_heater(config, gcode_id)
        # Setup kinematic checks
        self.nozzle_diameter = config.getfloat('nozzle_diameter', above=0.)
        filament_diameter = config.getfloat(
            'filament_diameter', minval=self.nozzle_diameter)
        self.filament_area = math.pi * (filament_diameter * .5)**2
        def_max_cross_section = 4. * self.nozzle_diameter**2
        def_max_extrude_ratio = def_max_cross_section / self.filament_area
        max_cross_section = config.getfloat(
            'max_extrude_cross_section', def_max_cross_section, above=0.)
        self.max_extrude_ratio = max_cross_section / self.filament_area
        logging.info("Extruder max_extrude_ratio=%.6f", self.max_extrude_ratio)
        toolhead = self.printer.lookup_object('toolhead')
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_e_velocity = config.getfloat(
            'max_extrude_only_velocity', max_velocity * def_max_extrude_ratio
            , above=0.)
        self.max_e_accel = config.getfloat(
            'max_extrude_only_accel', max_accel * def_max_extrude_ratio
            , above=0.)
        self.max_e_dist = config.getfloat(
            'max_extrude_only_distance', 50., minval=0.)
        self.instant_corner_v = config.getfloat(
            'instantaneous_corner_velocity', 1., minval=0.)
        
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
        
        # trapq for fiber extruder
        self.fiber_trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        
        self.fiber_extruder_stepper = self.printer.lookup_object('fiberextruder_stepper%d' % (extruder_num))
        self.fiber_extruder_stepper.stepper.set_trapq(self.fiber_trapq)
        
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        if self.name == 'fiberprinter':
            toolhead.set_extruder(self, 0.)
            gcode.register_command("M104", self.cmd_M104)
            gcode.register_command("M109", self.cmd_M109)
        gcode.register_mux_command("ACTIVATE_EXTRUDER", "EXTRUDER",
                                   self.name, self.cmd_ACTIVATE_EXTRUDER,
                                   desc=self.cmd_ACTIVATE_EXTRUDER_help)
    def update_move_time(self, flush_time, clear_history_time):
        self.trapq_finalize_moves(self.trapq, flush_time, clear_history_time)
    def get_status(self, eventtime):
        sts = self.heater.get_status(eventtime)
        sts['can_extrude'] = self.heater.can_extrude
        if self.extruder_stepper is not None:
            sts.update(self.extruder_stepper.get_status(eventtime))
        return sts
    def get_name(self):
        return self.name
    def get_heater(self):
        return self.heater
    def get_trapq(self):
        return self.trapq
    def stats(self, eventtime):
        return self.heater.stats(eventtime)
    def check_move(self, move):
        axis_r = move.axes_r[3]
        if not self.heater.can_extrude:
            raise self.printer.command_error(
                "Extrude below minimum temp\n"
                "See the 'min_extrude_temp' config option for details")
        if (not move.axes_d[0] and not move.axes_d[1]) or axis_r < 0.:
            # Extrude only move (or retraction move) - limit accel and velocity
            if abs(move.axes_d[3]) > self.max_e_dist:
                raise self.printer.command_error(
                    "Extrude only move too long (%.3fmm vs %.3fmm)\n"
                    "See the 'max_extrude_only_distance' config"
                    " option for details" % (move.axes_d[3], self.max_e_dist))
            inv_extrude_r = 1. / abs(axis_r)
            move.limit_speed(self.max_e_velocity * inv_extrude_r,
                             self.max_e_accel * inv_extrude_r)
        elif axis_r > self.max_extrude_ratio:
            if move.axes_d[3] <= self.nozzle_diameter * self.max_extrude_ratio:
                # Permit extrusion if amount extruded is tiny
                return
            area = axis_r * self.filament_area
            logging.debug("Overextrude: %s vs %s (area=%.3f dist=%.3f)",
                          axis_r, self.max_extrude_ratio, area, move.move_d)
            raise self.printer.command_error(
                "Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)\n"
                "See the 'max_extrude_cross_section' config option for details"
                % (area, self.max_extrude_ratio * self.filament_area))
    def calc_junction(self, prev_move, move):
        diff_r = move.axes_r[3] - prev_move.axes_r[3]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r))**2
        return move.max_cruise_v2
    def move(self, print_time, move):
        # extruder move
        if move.axes_d[3]:
            axis_r = move.axes_r[3]
            accel = move.accel * axis_r
            start_v = move.start_v * axis_r
            cruise_v = move.cruise_v * axis_r
            can_pressure_advance = False
            if axis_r > 0. and (move.axes_d[0] or move.axes_d[1]):
                can_pressure_advance = True
            # Queue movement (x is extruder movement, y is pressure advance flag)
            self.trapq_append(self.trapq, print_time,
                            move.accel_t, move.cruise_t, move.decel_t,
                            move.start_pos[3], 0., 0.,
                            1., can_pressure_advance, 0.,
                            start_v, cruise_v, accel)
            
            self.last_position = move.end_pos[3]
            
        # fiber extruder move
        if move.axes_d[4]:
            fiber_axis_r = move.axes_r[4]
            fiber_accel = move.accel * fiber_axis_r
            fiber_start_v = move.start_v * fiber_axis_r
            fiber_cruise_v = move.cruise_v * fiber_axis_r
                
            # Queue movement (x is extruder movement, y is pressure advance flag)
            self.fiber_trapq_append(self.fiber_trapq, print_time,
                                move.accel_t, move.cruise_t, move.decel_t,
                                move.start_pos[4], 0., 0.,
                                1., False, 0.,
                                fiber_start_v, fiber_cruise_v, fiber_accel)
            
            self.fiber_last_position = move.end_pos[4]
        
    def find_past_position(self, print_time):
        if self.extruder_stepper is None:
            return 0.
        return self.extruder_stepper.find_past_position(print_time)
    def cmd_M104(self, gcmd, wait=False):
        # Set Extruder Temperature
        temp = gcmd.get_float('S', 0.)
        index = gcmd.get_int('T', None, minval=0)
        if index is not None:
            section = 'fiberprinter'
            if index:
                section = 'fiberprinter%d' % (index,)
            extruder = self.printer.lookup_object(section, None)
            if extruder is None:
                if temp <= 0.:
                    return
                raise gcmd.error("Extruder not configured")
        else:
            extruder = self.printer.lookup_object('toolhead').get_extruder()
        pheaters = self.printer.lookup_object('heaters')
        pheaters.set_temperature(extruder.get_heater(), temp, wait)
    def cmd_M109(self, gcmd):
        # Set Extruder Temperature and Wait
        self.cmd_M104(gcmd, wait=True)
    cmd_ACTIVATE_EXTRUDER_help = "Change the active extruder"
    def cmd_ACTIVATE_EXTRUDER(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_extruder() is self:
            gcmd.respond_info("Extruder %s already active" % (self.name,))
            return
        gcmd.respond_info("Activating fiber extruder %s" % (self.name,))
        toolhead.flush_step_generation()
        toolhead.set_extruder(self, self.last_position)
        self.printer.send_event("extruder:activate_extruder")

def load_extruders(config):
    printer = config.get_printer()
    for i in range(99):
        section = 'fiberprinter'
        section_extruder = 'fiberextruder'
        if i:
            section = 'fiberprinter%d' % (i,)
            section_extruder = 'fiberextruder%d' % (i,)
            
        if not config.has_section(section) or not config.has_section(section_extruder):
            break
        
        pe = FiberExtruder(config.getsection(section), i)
        printer.add_object(section, pe)
        printer.add_object(section_extruder, pe)