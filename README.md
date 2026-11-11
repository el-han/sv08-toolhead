# SV08 toolhead: Sherpa Mini + Goliath

Conversion of the Sovol SV08 toolhead to a Sherpa Mini extruder (Annex) with an air-cooled
Mellow/VzBoT Goliath hotend, modelled in [build123d](https://github.com/gumyr/build123d).
Printed parts are designed for FDM in ASA.

The nozzle stays at its original position. The Sherpa keeps its own gears and drive geometry and is
driven by an LDO 36STH17 with a 10 tooth pinion. The original SV08 parts (carriage interface, Eddy probe,
fans, PCBs) are reused.

## Setup

```sh
git submodule update --init      # SV08, Sherpa Mini and Goliath source models in ext/
uv sync
```

## Usage

```sh
.venv/bin/python render.py        # printable parts + out/assembly.step (add --png for preview images)
.venv/bin/python check.py         # collision check (exit code 1 on collisions)
```

`out/` is generated and not tracked.

## Printed parts

| File (out/)                | Part        | Print orientation           |
|----------------------------|-------------|-----------------------------|
| `extruder_body_sherpa.step`| main body with Goliath carrier and fan wedges | back side on the bed |
| `pcb_holder.step`          | PCB holder with front bearing seat, idler pin seat and filament guides | flat, inside up |
| `motor_plate.step`         | motor plate with rear bearing seat and rear idler pin seat | flat |

## Hardware

| Qty | Part | Where |
|----:|------|-------|
| 8 | heat-set insert M3x5x4 (Ø4.8 hole) | 4 posts, 2 lugs, 1 PCB holder, 1 Goliath carrier (PCB) |
| 1 | heat-set insert M3x5x4 | idler thumbscrew (as in the Sherpa) |
| 4 | M3x12 DIN 912 | motor plate to PCB holder |
| 2 | M3x6 DIN 912 | motor |
| 2 | M3x8 DIN 912 | PCB holder lugs to main body |
| 2 | M3x12 | 2515 fan (cuts into the fan holes) |
| 4 | M2.5x5 DIN 912 | Goliath |

The Sherpa parts (gears, bearings, shafts, idler arm, thumbscrew, spring) are used as in the original.

## Code

| Module | Content |
|--------|---------|
| `common.py` | paths, filament axis, SV08 STEP loader, list of dropped SV08 parts |
| `sherpa.py` | Sherpa parts placed in SV08 coordinates (one rigid transformation) |
| `goliath_carrier.py` | carrier plate below the housing, Goliath placement, 2515 fan wedges |
| `housing.py` | PCB holder, motor plate, filament guides, clearances |
| `extruder_body.py` | main body (block with holes and pockets, fused with the carrier) |
| `fasteners.py` | screws and heat-set inserts (models from the SV08 and Sherpa STEPs) |
| `assembly.py` | placed parts and the named, coloured STEP tree |
| `render.py`, `check.py` | entry points |

Coordinates are those of the Sovol STEP assembly (mm): X to the right, Y to the back, Z up, seen from
the front of the printer.
