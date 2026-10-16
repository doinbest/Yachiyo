"""Run only when C compilation is allowed; no hardware commands."""
from run_firmware_tests import build_test

if __name__ == "__main__":
    build_test("chassis_imu_reference_test", "template/App/chassis_motion.c",
               "template/App/chassis_localization.c", "template/App/chassis_observer.c",
               "template/App/chassis_model.c")
    build_test("chassis_motion_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_heading_test", "template/App/chassis_motion.c",
               "template/App/chassis_localization.c", "template/App/chassis_observer.c",
               "template/App/chassis_model.c", "template/App/chassis_heading.c")
