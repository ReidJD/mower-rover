"""
yard_mapper_enclosure.py — Parametric CadQuery design for yard mapper hardware.

Generates 4 parts:
  enclosure_body.stl   — box that holds Raspberry Pi 4 + BNO055 IMU
  enclosure_lid.stl    — lid with sealing lip; secured by 4× M3 screws into top flange
  servo_mount.stl      — plate that bolts to enclosure bottom; holds MG996R shaft-down
  lidar_arm.stl        — attaches to servo horn; holds TFmini-S lens pointing down

SCANNING GEOMETRY:
  The enclosure mounts horizontally on the rear of the mower (saddle clamps on
  mower rail). The servo mount plate hangs below the enclosure with the MG996R
  output shaft pointing straight down. The lidar arm rotates in a horizontal plane
  ±60°, always pointing the TFmini-S lens straight at the ground. As the mower
  moves forward, successive sweeps build up parallel cross-track profiles.

HOW TO USE IN CQ-EDITOR:
  1. Download CQ-editor: https://github.com/CadQuery/CQ-editor/releases
  2. Open this file.  The enclosure body + lid are shown together by default.
  3. To preview other parts, edit the show_object() lines at the very bottom.
  4. Export: File → Export → STL.

COMMAND-LINE EXPORT (generates all 4 STL files):
  pip install cadquery
  python yard_mapper_enclosure.py

HARDWARE NOTES:
  Pi standoffs    — tap M2.5; secure Pi with M2.5×6 screws from below
  Lid flange      — tap M3 into flange holes; use M3×10 pan-head screws
                    apply 3 mm foam weatherstrip tape to the sealing lip
  USB-C power     — DWEII panel-mount USB-C (ASIN B0C7CPYL79) in 9 mm round hole
                    outside: buck converter USB-C cable plugs in
                    inside: 2-wire pigtail → GPIO pin 4 (5V) and pin 6 (GND)
  Cable glands    — 2× PG7 metric gland on the same short wall
                    PG7 fits cables 3–6.5 mm diameter; M12 threaded body
                    thread gland from outside, tighten nut from inside
                    Gland Y=0: servo signal + lidar UART cables (bundled)
                    Gland Y=+22: spare → RTK GPS UART in Phase 2
  BNO055 mount    — 2× M2 screws into the printed standoffs on the shelf
  Bottom mounts   — 4× M5 holes for standard pipe saddle clamps (hardware store)
  Servo mount     — 4× M3×8 screws through plate into enclosure bottom (tapped);
                    2× M4×16 screws through plate into MG996R mounting tabs
  Lidar arm       — press arm hub onto servo horn; 2× M2.5×6 screws to secure;
                    2× M2×8 screws through arm into TFmini-S mounting holes
  Cable routing   — TFmini-S cable exits upward, loops with ~100 mm slack near
                    the servo shaft to accommodate ±60° sweep, then enters gland

PRINT SETTINGS:
  Material: PETG (recommended) or ASA (better UV resistance for outdoor use)
  Layer height: 0.2 mm  |  Walls: 4 perimeters  |  Top/bottom: 5 layers
  Infill: 40% for servo mount + lidar arm, 20% for enclosure walls and lid
  Orientation:
    enclosure_body  — open top face UP on print bed
    enclosure_lid   — flat top face DOWN on print bed (lip points UP while printing)
    servo_mount     — flat (enclosure-side) face DOWN on print bed
    lidar_arm       — flat (servo-side) face DOWN on print bed
"""

import cadquery as cq
import os

# =============================================================================
# PARAMETERS
# =============================================================================

WALL     = 3.0    # enclosure wall thickness (mm)
CORNER_R = 3.0    # outer vertical-edge fillet radius

# ── Raspberry Pi 4 (bare board) ───────────────────────────────────────────────
PI_W = 85.6       # PCB length: USB-C / HDMI end → SD-card end
PI_D = 56.5       # PCB width:  USB-A side → Ethernet side
# Mounting hole centres, from the USB-C / USB-A corner of the PCB (mm)
PI_HOLES = [
    ( 3.5,  3.5),   # USB-C end, USB-A side
    ( 3.5, 52.5),   # USB-C end, Ethernet side
    (61.5,  3.5),   # SD-card end, USB-A side
    (61.5, 52.5),   # SD-card end, Ethernet side
]
STANDOFF_H  = 5.0   # M2.5 standoff height
STANDOFF_OD = 6.0   # standoff outer diameter
STANDOFF_ID = 2.7   # M2.5 tap drill

# ── Internal cavity ───────────────────────────────────────────────────────────
CLEAR_XY = 5.0      # clearance around Pi on all four sides

INT_W = PI_W + 2 * CLEAR_XY    # ~95.6 mm
INT_D = PI_D + 2 * CLEAR_XY    # ~66.5 mm
# INT_H breakdown: standoff(5) + PCB(1.5) + GPIO header(8.5) + cable routing(8) + headroom(10)
INT_H = 33.0

EXT_W = INT_W + 2 * WALL       # ~101.6 mm
EXT_D = INT_D + 2 * WALL       # ~72.5 mm
EXT_H = INT_H + WALL           # ~36 mm  (bottom wall + interior height)

# ── Top mounting flange ───────────────────────────────────────────────────────
# A flat frame around the opening at the top of the enclosure.
# The lid sits on this flange; 4× M3 screws through the lid thread into the flange.
# Foam weatherstrip tape on the lid sealing lip sits just inside the flange edge.
FLANGE_W = 9.0      # flange width (extends outward from enclosure exterior)
FLANGE_T = 5.0      # flange thickness; must be ≥ 5 mm for reliable M3 thread in PETG

# M3 screw hole positions in the flange (one near each corner)
FLANGE_HOLE_D   = 2.6     # M3 tap drill
FLANGE_HOLE_INS = 5.0     # hole centre inset from flange outer edge

# ── Lid ───────────────────────────────────────────────────────────────────────
LID_T     = 3.0     # lid plate thickness
LIP_DEPTH = 5.0     # sealing lip insert depth (fills flange thickness = snug fit)
LIP_T     = 2.5     # sealing lip wall thickness
LIP_GAP   = 0.4     # radial clearance between lip and cavity wall

# ── Panel-mount USB-C power connector ────────────────────────────────────────
# DWEII waterproof chassis-mount USB-C female socket (ASIN B0C7CPYL79).
# Mounts in a 9 mm round hole. Outside: buck converter USB-C cable plugs in.
# Inside: 2-wire pigtail (VBUS + GND) connects to Pi GPIO pin 4 (5V) and pin 6 (GND).
# No USB-C cable inside the box — zero clearance issues.
USBC_HOLE_D = 9.0     # panel-mount connector body diameter (9 mm as confirmed)
USBC_Y_POS  = -22.0   # Y position on wall (closest to Pi USB-C port location)

# ── Cable glands — PG7 metric (3–6.5 mm cable diameter) ──────────────────────
# Two PG7 glands on the −X SHORT wall (same wall as the USB-C connector).
# Purpose (inside → outside):
#   GLAND 1 (centre, Y =  0): servo signal wire + lidar UART cable (bundled)
#   GLAND 2 (right,  Y = +22): spare — will be used for RTK GPS UART in Phase 3
GLAND_BOSS_OD  = 16.0   # boss outer diameter on exterior wall face
GLAND_HOLE_D   = 12.0   # through-hole diameter (M12 thread for PG7 gland body)
GLAND_PROTRUDE =  5.0   # boss protrusion beyond exterior wall surface
GLAND_Y_POS    = [0.0, 22.0]   # two PG7 glands; 22 mm spacing from USB-C connector
GLAND_Z        = EXT_H / 2    # centred vertically on the wall

# ── BNO055 IMU shelf ──────────────────────────────────────────────────────────
# A raised shelf at the +X end (GPIO / SD-card end) of the enclosure interior.
# The BNO055 breakout sits on this shelf, above the Pi PCB level.
# Cables run down to the Pi GPIO header which is at this same end.
BNO_SHELF_W   = 28.0    # shelf width (X, along enclosure length)
BNO_SHELF_D   = 30.0    # shelf depth (Y)
BNO_SHELF_H   = STANDOFF_H + 3.5   # height above floor (clears Pi PCB + 2 mm)
BNO_SHELF_T   = 3.0     # shelf plate thickness
BNO_POST_OD   = 5.0     # BNO055 mounting post outer diameter
BNO_POST_ID   = 1.8     # M2 tap drill
BNO_POST_H    = 3.0     # post height above shelf plate
# BNO055 mounting hole spacing: ~18 mm C-C (adjust to match your breakout board)
BNO_HOLE_SPACING_W = 18.0   # C-C along X
BNO_HOLE_SPACING_D = 14.0   # C-C along Y

# ── Bottom mounting holes (for pipe saddle clamps) ────────────────────────────
# Four M5 holes in the bottom face, arranged in a rectangle.
# Use standard pipe/tube saddle clamps (hardware store) bolted through these holes.
MOUNT_HOLE_D = 5.2      # M5 clearance
MOUNT_HOLE_X = EXT_W / 2 - WALL - 4.0    # ±X positions of hole pairs
MOUNT_HOLE_Y = EXT_D / 2 - WALL - 4.0    # ±Y positions

# ── MG996R servo body dimensions ─────────────────────────────────────────────
SERVO_BODY_L      = 40.7    # body length (long axis)
SERVO_BODY_W      = 19.7    # body width
SERVO_FLANGE_SPAN = 47.5    # mounting tab hole C-C along body long axis
SERVO_HOLE_D      = 4.3     # M4 clearance for tab mounting screws

# ── Servo mount plate (Part 3) ────────────────────────────────────────────────
# Flat plate that bolts to the enclosure bottom face, holding the MG996R body
# with its output shaft pointing straight down.
SMOUNT_L          = 76.0    # plate length (aligned with servo body long axis)
SMOUNT_W          = 38.0    # plate width (across servo body)
SMOUNT_T          = 5.0     # plate thickness
SMOUNT_POCKET_L   = 41.5    # locating pocket for servo body (40.7 + 0.8 clearance)
SMOUNT_POCKET_W   = 20.5    # locating pocket for servo body (19.7 + 0.8 clearance)
SMOUNT_POCKET_D   = 2.5     # pocket depth (locates body; servo tabs rest on plate)
SMOUNT_SHAFT_D    = 13.0    # through-hole for servo shaft + horn base clearance
SMOUNT_SCREW_D    = 3.2     # M3 clearance: bolts plate to enclosure bottom
SMOUNT_SCREW_INS  = 7.0     # M3 hole inset from plate edge

# ── Lidar arm (TFmini-S ↔ servo horn adapter) ─────────────────────────────────
# The arm attaches to the servo horn at one end and holds the TFmini-S
# at the other end with the lens pointing STRAIGHT DOWN.
# TFmini-S body: 42 mm L × 15 mm W × 16 mm H. Lens on the bottom when mounted.
# Cable exits from the rear (short end), routes along the arm back to the gland.
# ⚠ Verify LIDAR_HOLE_SPACING against your specific TFmini-S unit before printing.
LIDAR_HOLE_SPACING = 36.0   # M2 mounting hole C-C on TFmini-S back face (verify!)
LIDAR_HOLE_D       = 2.2    # M2 clearance
LIDAR_BODY_L       = 42.0   # TFmini-S body length
LIDAR_BODY_W       = 15.0   # TFmini-S body width
LIDAR_BODY_H       = 16.0   # TFmini-S body height
HORN_HUB_D         = 6.3    # servo horn centre hub OD (+ 0.3 mm clearance)
HORN_SCREW_SPACING = 16.0   # horn screw C-C (4-arm cross horn, inner hole pair)
HORN_SCREW_TAP     = 2.7    # M2.5 tap
ARM_PLATE_T        = 4.0    # arm plate thickness
ARM_PLATE_W        = 50.0   # arm plate width (must span LIDAR_HOLE_SPACING + walls)
ARM_PLATE_L        = 65.0   # arm plate length: servo shaft centre → TFmini-S centre


# =============================================================================
# GEOMETRY HELPERS
# =============================================================================

def box_z(w, d, h, x=0.0, y=0.0, z=0.0):
    """Box centred at (x, y), bottom face at z."""
    return (
        cq.Workplane("XY")
        .transformed(offset=cq.Vector(x, y, z))
        .box(w, d, h, centered=(True, True, False))
    )

def cyl_z(r, h, x=0.0, y=0.0, z=0.0):
    """Vertical cylinder; base centre at (x, y, z)."""
    return (
        cq.Workplane("XY")
        .transformed(offset=cq.Vector(x, y, z))
        .circle(r)
        .extrude(h)
    )

def cyl_x(r, h, x=0.0, y=0.0, z=0.0):
    """Horizontal cylinder along +X; base centre at (x, y, z)."""
    return cq.Workplane("XY").add(
        cq.Solid.makeCylinder(r, h,
                              pnt=cq.Vector(x, y, z),
                              dir=cq.Vector(1, 0, 0))
    )

def cyl_y(r, h, x=0.0, y=0.0, z=0.0):
    """Horizontal cylinder along +Y; base centre at (x, y, z)."""
    return cq.Workplane("XY").add(
        cq.Solid.makeCylinder(r, h,
                              pnt=cq.Vector(x, y, z),
                              dir=cq.Vector(0, 1, 0))
    )


# =============================================================================
# PART 1: ENCLOSURE BODY
# Origin: centre of the bottom exterior face (Z = 0).
# Pi orientation: USB-C end at −X, USB-A side at −Y, GPIO at +X.
# =============================================================================

print("Building enclosure body...")

# ── Outer shell ───────────────────────────────────────────────────────────────
enc = (
    cq.Workplane("XY")
    .box(EXT_W, EXT_D, EXT_H, centered=(True, True, False))
    .edges("|Z")
    .fillet(CORNER_R)
)

# Carve out interior cavity from the top
enc = enc.cut(box_z(INT_W, INT_D, INT_H, z=WALL))

# ── Top mounting flange ───────────────────────────────────────────────────────
# Rectangular frame sitting on top of the enclosure walls.
# Outer: (EXT_W + 2×FLANGE_W) × (EXT_D + 2×FLANGE_W), thickness FLANGE_T.
# Inner opening matches cavity exactly so it doesn't block the Pi.
fl_outer_w = EXT_W + 2 * FLANGE_W
fl_outer_d = EXT_D + 2 * FLANGE_W
flange_outer = box_z(fl_outer_w, fl_outer_d, FLANGE_T, z=EXT_H)
flange_inner = box_z(INT_W, INT_D, FLANGE_T + 1, z=EXT_H - 0.5)    # cut out the opening
flange = flange_outer.cut(flange_inner)

# Round the flange outer corners
try:
    flange = flange.edges("|Z").fillet(CORNER_R)
except Exception:
    pass

enc = enc.union(flange)

# M3 tap holes in flange — one near each corner, centred between flange outer edge and cavity
fh_x = fl_outer_w / 2 - FLANGE_HOLE_INS    # ±X of hole centres
fh_y = fl_outer_d / 2 - FLANGE_HOLE_INS    # ±Y of hole centres
flange_screw_pos = [
    (-fh_x, -fh_y), (-fh_x, fh_y),
    ( fh_x, -fh_y), ( fh_x, fh_y),
]
for (fx, fy) in flange_screw_pos:
    enc = enc.cut(cyl_z(FLANGE_HOLE_D / 2, FLANGE_T + 1, fx, fy, EXT_H - 0.5))

# ── Pi mounting standoffs ─────────────────────────────────────────────────────
# Convert Pi hole coords (from PCB corner) to enclosure body-centred coords.
for (hx, hy) in PI_HOLES:
    cx = hx - PI_W / 2
    cy = hy - PI_D / 2
    enc = enc.union(cyl_z(STANDOFF_OD / 2, STANDOFF_H, cx, cy, WALL))
    enc = enc.cut(cyl_z(STANDOFF_ID / 2, WALL + STANDOFF_H + 1, cx, cy, -0.5))

# ── BNO055 IMU shelf and mounting posts ──────────────────────────────────────
# Shelf is at the +X end of the cavity (GPIO / SD-card end of Pi).
# Shelf X centre: at INT_W/2 − BNO_SHELF_W/2 (against the +X inner wall).
bno_shelf_x = INT_W / 2 - BNO_SHELF_W / 2
bno_shelf_z = WALL + BNO_SHELF_H   # top face of shelf = this Z

# Shelf plate — bridges between +X inner wall and a support column
shelf = box_z(BNO_SHELF_W, BNO_SHELF_D, BNO_SHELF_T, x=bno_shelf_x, z=bno_shelf_z)
# Support column under shelf (front edge, against +X wall)
support = box_z(WALL, BNO_SHELF_D, BNO_SHELF_H + BNO_SHELF_T,
                x=INT_W / 2 - WALL / 2, z=WALL)

enc = enc.union(shelf).union(support)

# Two M2 mounting posts on top of shelf for BNO055 screws
for (px, py) in [
    (bno_shelf_x - BNO_HOLE_SPACING_W / 2, -BNO_HOLE_SPACING_D / 2),
    (bno_shelf_x - BNO_HOLE_SPACING_W / 2, +BNO_HOLE_SPACING_D / 2),
]:
    enc = enc.union(cyl_z(BNO_POST_OD / 2, BNO_POST_H, px, py, bno_shelf_z + BNO_SHELF_T))
    enc = enc.cut(cyl_z(BNO_POST_ID / 2, BNO_POST_H + 1, px, py, bno_shelf_z + BNO_SHELF_T - 0.5))

# ── Cable gland bosses on −X wall (USB-C / power end) ────────────────────────
# Gland bosses are horizontal cylinders (axis along X) extruded outward.
# GLAND 1: power (USB-C from buck converter)
# GLAND 2: servo signal wire + lidar UART (bundle through one PG7)
# GLAND 2 (Y=+22): spare → RTK GPS UART in Phase 3
boss_x_start  = -EXT_W / 2 - GLAND_PROTRUDE
boss_x_length = GLAND_PROTRUDE + WALL + 0.5   # 0.5 overlap ensures clean union with wall

# Panel-mount USB-C connector — plain 9 mm hole through the wall (no boss)
enc = enc.cut(cyl_x(USBC_HOLE_D / 2, WALL + 1,
                     x=-EXT_W / 2 - 0.5, y=USBC_Y_POS, z=GLAND_Z))

# Two PG7 cable gland bosses (servo+lidar bundle, GPS spare)
for gy in GLAND_Y_POS:
    enc = enc.union(cyl_x(GLAND_BOSS_OD / 2, boss_x_length,
                           x=boss_x_start, y=gy, z=GLAND_Z))
    enc = enc.cut(cyl_x(GLAND_HOLE_D / 2, boss_x_length + 1,
                          x=boss_x_start - 0.5, y=gy, z=GLAND_Z))

# ── Bottom mounting holes (saddle clamps to mower rail) ───────────────────────
# Four M5 clearance holes for pipe saddle clamp bolts.
# Saddle clamps (available at any hardware store) attach the enclosure to the frame tube.
for (mx, my) in [
    (-MOUNT_HOLE_X, -MOUNT_HOLE_Y),
    (-MOUNT_HOLE_X,  MOUNT_HOLE_Y),
    ( MOUNT_HOLE_X, -MOUNT_HOLE_Y),
    ( MOUNT_HOLE_X,  MOUNT_HOLE_Y),
]:
    enc = enc.cut(cyl_z(MOUNT_HOLE_D / 2, WALL + 1, mx, my, -0.5))

# ── Bottom mounting holes (servo mount plate) ────────────────────────────────
# Four M3 tapped holes to receive the servo mount plate (Part 3).
# The plate is centred under the enclosure; holes are inset from plate edges.
smount_hx = SMOUNT_L / 2 - SMOUNT_SCREW_INS   # ±31 mm
smount_hy = SMOUNT_W / 2 - SMOUNT_SCREW_INS   # ±12 mm
for (sx, sy) in [
    (-smount_hx, -smount_hy), (-smount_hx, +smount_hy),
    (+smount_hx, -smount_hy), (+smount_hx, +smount_hy),
]:
    enc = enc.cut(cyl_z(SMOUNT_SCREW_D / 2, WALL + 1, sx, sy, -0.5))

print(f"  Body: {EXT_W:.1f}W × {EXT_D:.1f}D × {EXT_H + FLANGE_T:.1f}H mm (inc. flange)")
print(f"  Cavity: {INT_W:.1f}W × {INT_D:.1f}D × {INT_H:.1f}H mm")


# =============================================================================
# PART 2: ENCLOSURE LID
# Flat plate + downward sealing lip. The lid rests on the top flange.
# Origin: top face of lid at Z = 0; lid plate extends to Z = −LID_T;
#         sealing lip hangs below that to Z = −LID_T − LIP_DEPTH.
# =============================================================================

print("Building enclosure lid...")

# Lid plate — same footprint as the flange outer dimensions
lid_w = fl_outer_w
lid_d = fl_outer_d

lid = (
    cq.Workplane("XY")
    .box(lid_w, lid_d, LID_T, centered=(True, True, False))
    .translate((0, 0, -LID_T))
    .edges("|Z")
    .fillet(CORNER_R)
)

# Sealing lip — inserts into cavity opening; foam tape applied to lip perimeter
lip_ow = INT_W - LIP_GAP * 2
lip_od = INT_D - LIP_GAP * 2
lip_outer = box_z(lip_ow, lip_od, LIP_DEPTH, z=-LID_T - LIP_DEPTH)
lip_inner = box_z(lip_ow - LIP_T * 2, lip_od - LIP_T * 2,
                  LIP_DEPTH + 1, z=-LID_T - LIP_DEPTH - 0.5)
lid = lid.union(lip_outer.cut(lip_inner))

# M3 clearance holes aligning with flange tap holes
for (fx, fy) in flange_screw_pos:
    lid = lid.cut(cyl_z(FLANGE_HOLE_D / 2 + 0.4, LID_T + 1, fx, fy, -LID_T - 0.5))

print(f"  Lid: {lid_w:.1f}W × {lid_d:.1f}D mm, lip depth {LIP_DEPTH:.1f} mm")


# =============================================================================
# PART 3: SERVO MOUNT PLATE
#
# Flat plate that bolts flush to the underside of the enclosure.
# The MG996R servo body seats into a locating pocket with its output shaft
# pointing straight down through a clearance hole. The servo mounting tabs
# (which have M4 holes at SERVO_FLANGE_SPAN C-C) rest on the plate surface
# and are secured with M4×16 screws from below.
#
# Four M3 clearance holes at the plate corners bolt up into the tapped holes
# in the enclosure bottom face.
#
# Origin: centre of plate top face at Z = 0.
#         Plate extends downward (−Z). Servo shaft exits at Z = −SMOUNT_T.
# =============================================================================

print("Building servo mount plate...")

# ── Base plate ────────────────────────────────────────────────────────────────
servo_mount = box_z(SMOUNT_L, SMOUNT_W, SMOUNT_T, z=-SMOUNT_T)

# ── Locating pocket for servo body (top face) ────────────────────────────────
# The servo body drops into this pocket; the tabs sit proud on the plate surface.
servo_mount = servo_mount.cut(
    box_z(SMOUNT_POCKET_L, SMOUNT_POCKET_W, SMOUNT_POCKET_D + 0.5, z=-SMOUNT_POCKET_D - 0.5)
)

# ── Servo shaft clearance hole (centre of plate) ─────────────────────────────
servo_mount = servo_mount.cut(
    cyl_z(SMOUNT_SHAFT_D / 2, SMOUNT_T + 2, z=-SMOUNT_T - 1)
)

# ── M4 servo tab holes (through plate, along plate long axis) ─────────────────
for tx in [-SERVO_FLANGE_SPAN / 2, SERVO_FLANGE_SPAN / 2]:
    servo_mount = servo_mount.cut(
        cyl_z(SERVO_HOLE_D / 2, SMOUNT_T + 2, x=tx, z=-SMOUNT_T - 1)
    )

# ── M3 holes to bolt plate to enclosure bottom (corner positions) ─────────────
for (sx, sy) in [
    (-smount_hx, -smount_hy), (-smount_hx, +smount_hy),
    (+smount_hx, -smount_hy), (+smount_hx, +smount_hy),
]:
    servo_mount = servo_mount.cut(
        cyl_z(SMOUNT_SCREW_D / 2, SMOUNT_T + 2, x=sx, y=sy, z=-SMOUNT_T - 1)
    )

# ── Chamfer plate edges for printability ──────────────────────────────────────
try:
    servo_mount = servo_mount.edges("|Z").fillet(2.5)
except Exception:
    pass

print(f"  Servo mount plate: {SMOUNT_L:.0f}L × {SMOUNT_W:.0f}W × {SMOUNT_T:.0f}T mm")
print(f"  Shaft hole: {SMOUNT_SHAFT_D:.0f} mm dia  |  Tab holes: M4 at ±{SERVO_FLANGE_SPAN/2:.1f} mm")


# =============================================================================
# PART 4: LIDAR ARM
#
# Flat plate that rotates with the servo horn. The pivot end has a hub clearance
# hole and 2× M2.5 tap holes for the servo horn screws. The far end has a
# recessed cradle that the TFmini-S nestles into from below, lens pointing DOWN,
# secured by 2× M2 screws from the top of the arm.
#
# At nadir (servo 0°) the arm is horizontal and the TFmini-S lens aims straight
# at the ground. As the servo sweeps ±60° the arm rotates in the horizontal
# plane — the lens always points down, scanning a cross-track arc.
#
# Cable routing: TFmini-S cable exits toward the pivot end, runs along the
# top of the arm, loops ~100 mm at the servo shaft (slack for sweep), then
# enters the enclosure through the PG7 gland at Y = 0.
#
# Origin: servo shaft axis at (0, 0, 0). Plate extends from Y = −10 (behind
#         shaft) to Y = +ARM_PLATE_L. Top face of arm plate at Z = 0.
# =============================================================================

print("Building lidar arm...")

# The plate extends slightly behind the shaft for horn-screw access
arm_y_start = -10.0
arm_y_end   = ARM_PLATE_L

lidar_arm = box_z(ARM_PLATE_W, arm_y_end - arm_y_start, ARM_PLATE_T,
                  y=(arm_y_start + arm_y_end) / 2, z=-ARM_PLATE_T)

# ── Servo horn attachment (pivot end) ────────────────────────────────────────
# Hub clearance hole — servo horn centre hub fits through here
lidar_arm = lidar_arm.cut(
    cyl_z(HORN_HUB_D / 2, ARM_PLATE_T + 2, x=0, y=0, z=-ARM_PLATE_T - 1)
)
# 2× M2.5 tap holes (inner pair on a 4-arm cross horn, along X axis)
for hx in [-HORN_SCREW_SPACING / 2, HORN_SCREW_SPACING / 2]:
    lidar_arm = lidar_arm.cut(
        cyl_z(HORN_SCREW_TAP / 2, ARM_PLATE_T + 2, x=hx, y=0, z=-ARM_PLATE_T - 1)
    )

# ── TFmini-S cradle (far end) ─────────────────────────────────────────────────
# The TFmini-S body (42×15×16 mm) sits in a shallow pocket in the BOTTOM face
# of the arm, lens pointing down. The cradle prevents lateral movement.
# M2 screws from the TOP of the arm pass through and thread into the sensor.
lidar_cy = ARM_PLATE_L   # Y position of TFmini-S centre

# Shallow locating pocket in bottom face (1.5 mm deep, body footprint + 0.5 clearance)
lidar_arm = lidar_arm.cut(
    box_z(LIDAR_BODY_L + 0.5, LIDAR_BODY_W + 0.5, 1.5 + 0.5,
          y=lidar_cy, z=-ARM_PLATE_T - 0.5)
)

# 2× M2 clearance holes for mounting screws (top to bottom, at LIDAR_HOLE_SPACING C-C)
for lx in [-LIDAR_HOLE_SPACING / 2, LIDAR_HOLE_SPACING / 2]:
    lidar_arm = lidar_arm.cut(
        cyl_z(LIDAR_HOLE_D / 2, ARM_PLATE_T + 2, x=lx, y=lidar_cy, z=-ARM_PLATE_T - 1)
    )

# ── Cable channel on top face ────────────────────────────────────────────────
# Shallow groove that guides the TFmini-S cable from the sensor end toward pivot.
# Cable sits in groove and is zip-tied at two points to keep it tidy.
lidar_arm = lidar_arm.cut(
    box_z(4.0, ARM_PLATE_L - 20.0, 1.5 + 0.5,
          y=(arm_y_start + ARM_PLATE_L - 20.0) / 2 + 10, z=-1.5 - 0.5)
)

# ── Stiffening rib along the length (underside, centre) ──────────────────────
rib_h = 4.0
lidar_arm = lidar_arm.union(
    box_z(5.0, arm_y_end - arm_y_start - 30.0, rib_h,
          y=(arm_y_start + arm_y_end) / 2, z=-ARM_PLATE_T - rib_h)
)

# Round corners
try:
    lidar_arm = lidar_arm.edges("|Z").fillet(2.0)
except Exception:
    pass

print(f"  Lidar arm: {ARM_PLATE_W:.0f}W × {ARM_PLATE_L + 10:.0f}L mm  "
      f"(pivot→sensor: {ARM_PLATE_L:.0f} mm)")


# =============================================================================
# COMMAND-LINE EXPORT
# =============================================================================

if __name__ == "__main__":
    out = os.path.dirname(os.path.abspath(__file__))
    for name, part in [("enclosure_body", enc), ("enclosure_lid", lid),
                        ("servo_mount", servo_mount), ("lidar_arm", lidar_arm)]:
        path = os.path.join(out, f"{name}.stl")
        cq.exporters.export(part, path)
        print(f"Exported: {path}")


# =============================================================================
# CQ-EDITOR — edit show_object() calls to preview different parts.
# Default: enclosure body + lid floated above.
# Uncomment servo_mount / lidar_arm lines to preview the scanning assembly.
# =============================================================================

show_object(enc,
            name="enclosure_body",
            options={"color": "#3a6ebf", "alpha": 0.85})

show_object(lid.translate((0, 0, EXT_H + FLANGE_T + 5)),  # float lid above body
            name="enclosure_lid",
            options={"color": "#5aaa6a", "alpha": 0.85})

# Uncomment to preview servo mount plate (hangs below enclosure body):
# show_object(servo_mount.translate((0, 0, -SMOUNT_T - 2)),
#             name="servo_mount", options={"color": "#bf8c3a", "alpha": 0.9})

# Uncomment to preview lidar arm (positioned below servo mount):
# show_object(lidar_arm.translate((0, 0, -SMOUNT_T - 20)),
#             name="lidar_arm", options={"color": "#bf3a6e", "alpha": 0.9})
