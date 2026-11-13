"""Shared paths, constants and the SV08 STEP loader.

Coordinates = coordinate system of the Sovol STEP assembly (mm).
"""
from functools import cache
from pathlib import Path

from build123d import Shape, import_step

from geometry import XY

ROOT = Path(__file__).parent
SV08_STEP = ROOT / "ext/SV08/STEP/JXHSV08-06000 挤出喷头组件（行星）hotend assembly planetary.step"
GOLIATH_STEP = ROOT / "ext/Goliath/CAD/Goliath Mockup AIR.STEP"
SHERPA_STEP = ROOT / "ext/Sherpa_Mini-Extruder/CAD/sherpa_mini_release_2_rev1.STEP"
OUT = ROOT / "out"

FILAMENT_AXIS = XY(-335.70, 225.05)     # filament axis = nozzle position (original, unchanged)
ORIGINAL_NOZZLE_Z = -224.07             # original nozzle tip (unchanged)
HEAT_INSERT_HOLE = 4.0                  # heat-set insert M3x5x4: Ø4 hole, 6.7 mm deep
HEAT_INSERT_HOLE_DEPTH = 6.7
CLEARANCE = 0.4                         # default clearance of cut-outs around neighbouring parts


@cache
def sv08_parts() -> list[tuple[str, Shape]]:
    """(label, shape) for all parts of the SV08 STEP, including parts with the same label (e.g. 2x M3x16)."""
    assembly = import_step(str(SV08_STEP))
    return [(child.label, child) for child in assembly.children]


def find_sv08(key: str) -> list[Shape]:
    """All SV08 parts whose label contains `key`."""
    hits = [shape for label, shape in sv08_parts() if key in label]
    assert hits, key
    return hits


# Label fragments of the SV08 parts dropped by the Goliath conversion
REMOVED_BY_GOLIATH = ["xtruder_install_plate", "ooling_aluminum_block", "eating_block", "heating_plate",
                      "thermistor", "silicone_sock", "shoulder_screw", "M2_5X30", "M2_5X18", "wire_harness",
                      "ptfe_tube"]

# ... and additionally by the Sherpa conversion (planetary extruder, original motor, handle)
REMOVED_BY_SHERPA = REMOVED_BY_GOLIATH + [
    "gear_case", "planet_carrier", "cylindrical_gear", "extrusion_gear", "gear_shaft", "handle", "insulated_column",
    "36_step_motor", "motor_knob", "set_screw", "bearing", "M3x16", "knurled_hand_screw", "magnet_ring",
    "wire_clip_plate", "M3x12__hex_socket", "stainless_steel_spring", "flange15_6mm", "ptfe_tube_quick",
    "plain_washer", "_magnet",                      # magnets are dropped together with the front cover
]


def is_removed_sv08_part(label: str) -> bool:
    return any(key in label for key in REMOVED_BY_SHERPA)
