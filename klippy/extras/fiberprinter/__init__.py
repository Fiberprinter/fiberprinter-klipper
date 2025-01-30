from extras.fiberprinter.fibercutter import CutterServo
from extras.fiberprinter.fiberextruder import load_extruders

class FiberPrinter:
    def __init__(self, config):
        self.cutter = CutterServo(config.getsection('fiberprinter'))
        load_extruders(config)
        
def add_fiber_printer(config):
    return FiberPrinter(config)