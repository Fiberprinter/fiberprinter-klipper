from extras.fiberprinter.fibercutter import CutterServo
from extras.fiberprinter.fiberextruder import load_extruders

class FiberPrinter:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.toolhead = printer.lookup_object('toolhead')
        self.cutter = CutterServo(config.getsection('fiberprinter'))
        
        load_extruders(config)
        
def load_config(config):
    return FiberPrinter(config)