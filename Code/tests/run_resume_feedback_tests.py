"""Explicit host regressions for mission resume; run when compilation is allowed."""
from run_firmware_tests import build_test

if __name__ == "__main__":
    build_test("contest_resume_feedback_test", "template/App/contest_mission.c", "template/Hardware/QR.c")
    build_test("chassis_resume_readiness_test", "template/App/chassis_route.c", "template/App/chassis_route_profile.c")
