"""Generate the browser's grab parameter metadata from the firmware definition."""
from pathlib import Path
import argparse
import json
import re

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "template/App/grab_parameters.def"
OUTPUT = ROOT / "upper-computer-web/local-car/grab-parameter-metadata.mjs"


def render():
    parameters, defaults, track_defaults, track_parameters, comparison_defaults = {}, {}, {}, {}, {}
    for line in SOURCE.read_text(encoding="utf-8").splitlines():
        if line.startswith("TRACK_COMPARISON_DEFAULT("):
            match = re.fullmatch(r'TRACK_COMPARISON_DEFAULT\((\w+), "(\w+)", ([^,]+)\)', line)
            if match is None:
                raise ValueError(f"Invalid tracking comparison default: {line}")
            _, key, default = match.groups()
            comparison_defaults[key] = float(default)
            continue
        if line.startswith("TRACK_PARAM("):
            match = re.fullmatch(r'TRACK_PARAM\((\w+), "(\w+)", ([^,]+), ([^,]+), ([^,]+), (\d+)\)', line)
            if match is None:
                raise ValueError(f"Invalid tracking parameter definition: {line}")
            _, key, default, minimum, maximum, integer = match.groups()
            track_defaults[key] = float(default)
            track_parameters[key] = dict(min=float(minimum), max=float(maximum), step=1 if int(integer) else "any")
            continue
        if not line.startswith("PARAM("):
            continue
        fields = re.fullmatch(r'PARAM\((\w+), "(\w+)", ([^,]+), ([^,]+), ([^,]+), (\d+), (\d+)\)', line)
        if fields is None:
            raise ValueError(f"Invalid parameter definition: {line}")
        _, key, default, minimum, maximum, flags, phase = fields.groups()
        flags, phase = int(flags), int(phase)
        minimum = float(minimum)
        parameters[key] = dict(min=minimum,
                               max=float(maximum), step=1 if flags & 1 else "any")
        if flags & 2:
            parameters[key]["exclusiveMin"] = True
        if phase:
            defaults[key] = float(default)
    return ("// Generated from Code/template/App/grab_parameters.def; do not edit.\n"
            "// Defaults include values installed after verified homing; not device readback.\n"
            "export const GRAB_PARAMETER_METADATA = Object.freeze(" +
            json.dumps(parameters, ensure_ascii=False, indent=2) + ");\n"
            "export const GRAB_FORM_DEFAULTS = Object.freeze(" +
            json.dumps(defaults, ensure_ascii=False, indent=2) + ");\n" +
            "export const TRACK_PARAMETER_DEFAULTS = Object.freeze(" + json.dumps(track_defaults, indent=2) + ");\n" +
            "export const TRACK_COMPARISON_DEFAULTS = Object.freeze({...TRACK_PARAMETER_DEFAULTS, ..." + json.dumps(comparison_defaults, indent=2) + "});\n" +
            "export const TRACK_PARAMETER_METADATA = Object.freeze(" + json.dumps(track_parameters, indent=2) + ");\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Fail if browser metadata is stale")
    args = parser.parse_args()
    expected = render()
    if args.check:
        if not OUTPUT.exists() or OUTPUT.read_text(encoding="utf-8") != expected:
            raise SystemExit("Grab metadata stale: run python Code/tools/sync_grab_parameters.py")
        print("Grab parameter metadata matches firmware")
    else:
        OUTPUT.write_text(expected, encoding="utf-8", newline="\n")
        print(f"Generated {OUTPUT}")
