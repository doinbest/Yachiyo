from __future__ import annotations

import pathlib
import struct
import sys


PROJECT_ROOT = pathlib.Path(__file__).resolve().parents[1]
LOCAL_TEST_DEPS = PROJECT_ROOT / ".codex-test-deps"
LEGACY_TEST_DEPS = (
    PROJECT_ROOT
    / ".superpowers"
    / "sdd"
    / "2026-09-29-stm32-standalone-navigation"
    / "python-deps"
)
TEST_DEPS = LOCAL_TEST_DEPS if LOCAL_TEST_DEPS.exists() else LEGACY_TEST_DEPS
sys.path.insert(0, str(TEST_DEPS))

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_HOOK_CODE, UC_MODE_LITTLE_ENDIAN, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_SP


FLASH_BASE = 0x08000000
FLASH_SIZE = 0x00100000
RAM_BASE = 0x20000000
RAM_SIZE = 0x00010000
MAX_INSTRUCTIONS = 10_000_000


def load_symbols(elf: ELFFile) -> dict[str, int]:
    table = elf.get_section_by_name(".symtab")
    if table is None:
        raise RuntimeError("AXF has no symbol table")
    return {symbol.name: int(symbol.entry.st_value) for symbol in table.iter_symbols()}


def main() -> int:
    axf_path = (
        pathlib.Path(sys.argv[1]).resolve()
        if len(sys.argv) > 1
        else pathlib.Path(__file__).with_name("navigation_tests.axf")
    )
    with axf_path.open("rb") as stream:
        elf = ELFFile(stream)
        symbols = load_symbols(elf)
        required = (
            "main",
            "test_finished",
            "g_test_checks",
            "g_test_failures",
            "g_test_failure_lines",
        )
        missing = [name for name in required if name not in symbols]
        if missing:
            raise RuntimeError(f"missing AXF symbols: {', '.join(missing)}")

        emulator = Uc(
            UC_ARCH_ARM,
            UC_MODE_THUMB | UC_MODE_MCLASS | UC_MODE_LITTLE_ENDIAN,
        )
        emulator.mem_map(FLASH_BASE, FLASH_SIZE)
        emulator.mem_map(RAM_BASE, RAM_SIZE)

        for segment in elf.iter_segments():
            if segment.header.p_type != "PT_LOAD":
                continue
            address = int(segment.header.p_paddr or segment.header.p_vaddr)
            data = segment.data()
            if data:
                emulator.mem_write(address, data)
            zero_count = int(segment.header.p_memsz) - len(data)
            if zero_count > 0:
                emulator.mem_write(address + len(data), bytes(zero_count))

    finish_address = symbols["test_finished"] & ~1
    reached_finish = False

    def stop_at_finish(uc: Uc, address: int, _size: int, _data: object) -> None:
        nonlocal reached_finish
        if address == finish_address:
            reached_finish = True
            uc.emu_stop()

    emulator.hook_add(UC_HOOK_CODE, stop_at_finish)
    emulator.reg_write(UC_ARM_REG_SP, RAM_BASE + RAM_SIZE - 16)
    emulator.emu_start(symbols["main"] | 1, 0, count=MAX_INSTRUCTIONS)

    if not reached_finish:
        raise RuntimeError("test program did not reach test_finished")

    checks = struct.unpack(
        "<I", emulator.mem_read(symbols["g_test_checks"], 4)
    )[0]
    failures = struct.unpack(
        "<I", emulator.mem_read(symbols["g_test_failures"], 4)
    )[0]
    print(f"NAV_TEST_CHECKS={checks} NAV_TEST_FAILURES={failures}")
    if failures:
        stored = min(failures, 16)
        lines = struct.unpack(
            f"<{stored}I",
            emulator.mem_read(symbols["g_test_failure_lines"], stored * 4),
        )
        print("NAV_TEST_FAILURE_LINES=" + ",".join(str(line) for line in lines))
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
