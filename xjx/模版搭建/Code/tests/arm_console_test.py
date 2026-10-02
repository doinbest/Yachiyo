"""Focused regression checks for the USART1 arm console command surface."""
from pathlib import Path
import re


SOURCE = (Path(__file__).parents[1] / "template/App/arm_console.c").read_text(
    encoding="utf-8"
)


def command_body(name: str) -> str:
    match = re.search(
        rf"static uint8_t {name}\([^)]*\)\s*\{{(.*?)\n\}}",
        SOURCE,
        re.S,
    )
    assert match, f"missing {name}"
    return match.group(1)


def test_command_groups_are_split():
    for name in (
        "ArmConsole_GripCommandHandle",
        "ArmConsole_VisionCommandHandle",
        "ArmConsole_ChassisCommandHandle",
        "ArmConsole_MechanicalArmCommandHandle",
    ):
        assert name in SOURCE
    assert len(command_body("ArmConsole_CommandExecute").splitlines()) < 180


def test_status_commands_and_snapshot_trace_exist():
    for command in ("camera status", "imu status", "screen status"):
        assert command in SOURCE
    assert "Camera_SnapshotGet(&Snapshot)" in SOURCE


def test_unsupported_b3_is_explicit_and_not_advertised():
    help_body = SOURCE[SOURCE.index("static void ArmConsole_HelpShow"):SOURCE.index("static void ArmConsole_ConfigShow")]
    assert "vision ring <" not in help_body
    assert '"ERR Unsupported B3\\r\\n"' in SOURCE


if __name__ == "__main__":
    test_command_groups_are_split()
    test_status_commands_and_snapshot_trace_exist()
    test_unsupported_b3_is_explicit_and_not_advertised()
    print("arm_console_test: OK")
