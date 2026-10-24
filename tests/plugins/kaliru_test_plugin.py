# kaliru plugin layer test plugin (tests/cases/plugins/): reads config values
# with python defaults, registers a command, an event handler and a timer,
# pauses the reactor inside its command, logs, raises a command error
import logging


class TestPlugin:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()
        self.value = config.getfloat("value", 1.5, above=0.0)
        self.missing = config.getfloat("missing", None)
        self.count = config.getint("count", 3, minval=1)
        self.label = config.get("label", "none")
        self.flag = config.getboolean("flag", False)
        self.ticks = 0
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "TEST_PLUGIN", self.cmd_TEST_PLUGIN, desc="kaliru plugin test"
        )
        self.printer.register_event_handler("klippy:ready", self._ready)
        logging.info(
            "test plugin %s: value=%r missing=%r count=%r label=%r flag=%r",
            self.name, self.value, self.missing, self.count, self.label,
            self.flag,
        )

    def _ready(self):
        self.timer = self.reactor.register_timer(self._tick, self.reactor.NOW)

    def _tick(self, eventtime):
        self.ticks += 1
        return self.reactor.NEVER

    def cmd_TEST_PLUGIN(self, gcmd):
        v = gcmd.get_float("V", self.value, minval=0.0)
        n = gcmd.get_int("N", None)
        gcmd.respond_info("test plugin %s v=%.3f n=%r" % (self.name, v, n))
        self.reactor.pause(self.reactor.monotonic() + 0.05)
        gcmd.respond_info("after pause: %s ticks=%d" % (gcmd.get_command(), self.ticks))
        if gcmd.get_int("FAIL", 0):
            raise gcmd.error("test plugin failure")

    def get_status(self, eventtime):
        return {"value": self.value, "label": self.label}


def load_config(config):
    return TestPlugin(config)
