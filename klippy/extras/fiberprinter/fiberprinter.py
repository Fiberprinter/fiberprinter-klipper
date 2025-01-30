from fibercutter import CutterServo
import fiberextruder

class FiberPrinter:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.toolhead = printer.lookup_object('toolhead')
        self.cutter = CutterServo(config.getsection('fiberprinter'))
        
        fiberextruder.load_extruders(config)
        
        gcode = printer.lookup_object('gcode')
        
        gcode.register_command("G5", self.cmd_G5)
        
        
    def cmd_G5(self, gcmd):
        # Move
        params = gcmd.get_command_parameters()
        try:
            for pos, axis in enumerate('XYZ'):
                if axis in params:
                    v = float(params[axis])
                    if not self.absolute_coord:
                        # value relative to position of last move
                        self.last_position[pos] += v
                    else:
                        # value relative to base coordinate position
                        self.last_position[pos] = v + self.base_position[pos]
            if 'E' in params:
                v = float(params['E']) * self.extrude_factor
                if not self.absolute_coord or not self.absolute_extrude:
                    # value relative to position of last move
                    self.last_position[3] += v
                else:
                    # value relative to base coordinate position
                    self.last_position[3] = v + self.base_position[3]
            
            if 'D' in params:
                v = float(params['D']) * self.extrude_factor
                
                if not self.absolute_coord or not self.absolute_extrude:
                    # value relative to position of last move
                    self.last_position[4] += v
                else:
                    # value relative to base coordinate position
                    self.last_position[4] = v + self.base_position[3]  
            
            if 'F' in params:
                gcode_speed = float(params['F'])
                if gcode_speed <= 0.:
                    raise gcmd.error("Invalid speed in '%s'"
                                     % (gcmd.get_commandline(),))
                self.speed = gcode_speed * self.speed_factor
        except ValueError as e:
            raise gcmd.error("Unable to parse move '%s'"
                             % (gcmd.get_commandline(),))
        # self.move_with_transform(self.last_position, self.speed)
        
        move = fiberextruder.FiberMove(self, [], self.last_position, self.speed)
        if not move.move_d:
            return
        if move.is_kinematic_move:
            self.kin.check_move(move)
        if move.axes_d[3]:
            self.extruder.check_move(move)
        self.commanded_pos[:] = move.end_pos
        self.lookahead.add_move(move)
        if self.print_time > self.need_check_pause:
            self._check_pause()
    
    
    