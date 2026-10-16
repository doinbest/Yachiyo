"""Explicit entry for use when C compilation is allowed; never touches hardware."""
from run_firmware_tests import BASE, build_test, run, OUT

if __name__ == "__main__":
    # Preserve production's 2 ms handover; generic legacy fixtures disable it.
    run([*BASE, "tests/motor_config_stop_test.c", "template/Hardware/motor_bus.c",
         "-lm", "-o", OUT / "motor_config_stop_test.exe"])
    for case in ("before_tx", "after_tx", "verify", "missing", "write", "hold_queued", "hold_active", "recovery"):
        run([OUT / "motor_config_stop_test.exe", case])
    build_test("motor_bus_lazy_config_test", "template/Hardware/motor_bus.c",
               "template/Hardware/Emm_V5.c", "template/App/mechanical_arm.c",
               "template/App/turntable.c", "-Wno-old-style-declaration")
    build_test("motor_recovery_test", "template/Hardware/motor_bus.c")
    build_test("contest_recovery_trigger_test", "template/App/contest_mission.c", "template/Hardware/QR.c")
