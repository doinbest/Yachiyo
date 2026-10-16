"""None wheel controls with real queries, recovery and mechanism compatibility."""
from run_firmware_tests import build_test,run,BASE,OUT
GRAB=("template/App/grab_tracking.c","template/App/chassis_observer.c","template/App/chassis_model.c")
if __name__=="__main__":
    build_test("chassis_none_bus_test","template/Hardware/motor_bus.c","template/App/mecanum_chassis.c")
    run([OUT/"chassis_none_bus_test.exe","receive"])
    run([*BASE,"tests/wheel_none_config_stop_test.c","template/Hardware/motor_bus.c","-lm","-o",OUT/"wheel_none_config_stop_test.exe"])
    for case in ("normal","mismatch","missing"):run([OUT/"wheel_none_config_stop_test.exe",case])
    run([*BASE,"tests/motor_config_stop_test.c","template/Hardware/motor_bus.c","-lm","-o",OUT/"motor_config_stop_test.exe"])
    for case in ("before_tx","after_tx","verify","missing","write","hold_queued","hold_active","recovery"):
        run([OUT/"motor_config_stop_test.exe",case])
    build_test("motor_bus_reply_settle_test")
    build_test("motor_arrival_test","template/Hardware/motor_bus.c")
    build_test("motor_bus_lazy_config_test","template/Hardware/motor_bus.c","template/Hardware/Emm_V5.c",
               "template/App/mechanical_arm.c","template/App/turntable.c","-Wno-old-style-declaration")
    for name in ("grab_none_profile_test","grab_recovery_stream_test","grab_feedback_schedule_test","grab_arrival_fallback_test",
                 "grab_car_pick_test","grab_first_tray_clearance_test","grab_alignment_handoff_test","grab_bus_recovery_test"):
        build_test(name,*GRAB)
    build_test("grab_static_frame_test")
    build_test("turntable_feedback_schedule_test")
    build_test("turntable_bus_fairness_test","template/App/turntable.c","template/Hardware/motor_bus.c")
    print("chassis None focused suite:27 cases PASS")
