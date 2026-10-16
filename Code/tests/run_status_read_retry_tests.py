"""Run only when C compilation is authorized; no device or serial operations."""
from run_firmware_tests import BASE, OUT, run

if __name__ == "__main__":
    # Retain the production 2 ms handover, unlike legacy build_test defaults.
    run([*BASE, "tests/motor_status_retry_test.c", "template/Hardware/motor_bus.c",
         "-lm", "-o", OUT / "motor_status_retry_test.exe"])
    for case in ("async", "late", "stop", "normal", "reject", "missing", "fast", "busy",
                 "handover", "stop_handover",
                 "split_handover", "stale_partial", "partial_urgent", "clock_irq",
                 "speed", "position", "home_flags", "arm_state", "recovery_state",
                 "motion", "fd", "home", "config_read", "config_write"):
        run([OUT / "motor_status_retry_test.exe", case])
    run([*BASE, "tests/mecanum_status_retry_test.c", "template/App/mecanum_chassis.c",
         "template/Hardware/motor_bus.c", "template/App/chassis_model.c", "-lm",
         "-o", OUT / "mecanum_status_retry_test.exe"])
    run([OUT / "mecanum_status_retry_test.exe"])
