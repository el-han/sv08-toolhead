"""Shared paths, constants and loaders for the Sherpa/Goliath conversion.

Coordinates = coordinate system of the Sovol STEP assembly (mm).
"""
import re
from functools import lru_cache
from pathlib import Path

from build123d import import_step

ROOT = Path(__file__).parent
SV08_STEP = ROOT / "ext/SV08/STEP/JXHSV08-06000 挤出喷头组件（行星）hotend assembly planetary.step"
GOLIATH_STEP = ROOT / "ext/Goliath/CAD/Goliath Mockup AIR.STEP"
SHERPA_STEP = ROOT / "ext/Sherpa_Mini-Extruder/CAD/sherpa_mini_release_2_rev1.STEP"
OUT = ROOT / "out"

AX_X, AX_Y = -335.70, 225.05     # filament axis / nozzle (original, unchanged)
ORIG_NOZZLE_Z = -224.07          # original nozzle tip (unchanged)


def short(label):
    m = re.findall(r"[A-Za-z0-9][A-Za-z0-9_\-]{3,}", label or "")
    return (m[-1] if m else label)[:40]


@lru_cache(None)
def sv08_parts():
    """List of (label, shape) for all parts, including parts with the same label (e.g. 2x M3x16)."""
    asm = import_step(str(SV08_STEP))
    return [(c.label, c) for c in asm.children]


def find(key):
    hits = [p for l, p in sv08_parts() if key in l]
    assert hits, key
    return hits


# Parts already dropped by the Goliath conversion (basis of the flat plate back side)
REMOVED_ORIG = ["xtruder_install_plate", "ooling_aluminum_block", "eating_block", "heating_plate", "thermistor",
                "silicone_sock", "shoulder_screw", "M2_5X30", "M2_5X18", "wire_harness", "ptfe_tube"]

# Parts additionally dropped by the Sherpa conversion (planetary extruder, original motor, handle)
REMOVED_SHERPA = REMOVED_ORIG + [
    "gear_case", "planet_carrier", "cylindrical_gear", "extrusion_gear", "gear_shaft", "handle", "insulated_column",
    "36_step_motor", "motor_knob", "set_screw", "bearing", "M3x16", "knurled_hand_screw", "magnet_ring",
    "wire_clip_plate", "M3x12__hex_socket", "stainless_steel_spring", "flange15_6mm", "ptfe_tube_quick",
    "plain_washer", "_magnet",                      # magnets are dropped together with the front cover
]
