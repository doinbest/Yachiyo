"""Compile production C fixtures and check their real JSON against the browser model."""
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / ".embeddedskills" / "tests"
BASE = ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-Itests/usb_stubs", "-Itemplate/Hardware", "-Itemplate/App"]


def fixture(name, sources):
    exe = OUT / (name + ".exe")
    subprocess.run(BASE + ["tests/" + name + ".c", *sources, "-lm", "-o", str(exe)], cwd=ROOT, check=True)
    return subprocess.check_output([str(exe)], cwd=ROOT, text=True, encoding="utf-8")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    model = fixture("chassis_model_fixture", ["template/App/chassis_model.c"])
    telemetry = fixture("chassis_telemetry_test", ["template/App/chassis_telemetry.c", "template/App/chassis_localization.c", "template/App/chassis_observer.c", "template/App/chassis_model.c"])
    data = {"model": [json.loads(line) for line in model.splitlines() if line.startswith("{")],
            "telemetry": [line for line in telemetry.splitlines() if line.startswith("@CHASSIS")]}
    assert len(data["model"]) == 9 and len(data["telemetry"]) >= 2
    payload = json.dumps(data)
    (OUT / "chassis-map-fixtures.json").write_text(payload + "\n", encoding="utf-8")
    result = subprocess.run(["node", "upper-computer-web/local-car/tests/map-contract.mjs"], cwd=ROOT,
                            input=payload, text=True, encoding="utf-8", stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (OUT / "chassis-map-contract.log").write_text(result.stdout, encoding="utf-8")
    print(result.stdout, end="")
    result.check_returncode()


if __name__ == "__main__":
    main()
