"""
yard_mapper_enclosure.py — Parametric CadQuery design for yard mapper hardware.

Generates 4 parts:
  enclosure_body.stl  — box that holds Raspberry Pi 4 + BNO055 IMU
  enclosure_lid.stl   — lid with sealing lip; secured by 4× M3 screws into top flange
  servo_bracket.stl   — clamps to mower frame tube; positions MG996R servo
  lidar_arm.stl       — attaches TFmini-S lidar to servo horn

HOW TO USE IN CQ-EDITOR:
  1. Download CQ-editor: https://github.com/CadQuery/CQ-editor/releases
  2. Open this file.  The enclosure body + lid are shown together by default.
  3. To preview other parts, edit the show_object() lines at the very bottom.
  4. Export: File → Export → STL.

COMMAND-LINE EXPORT (generates all 4 STL files):
  pip install cadquery
  python yard_mapper_enclosure.py

HARDWARE NOTES:
  Pi standoffs   — tap M2.5; secure Pi with M2.5×6 screws from below
  Lid flange     — tap M3 into flange holes; use M3×10 pan-head screws
                   apply 3 mm foam weatherstrip tape to the sealing lip
  USB-C power    — DWEII panel-mount USB-C (ASIN B0C7CPYL79) in 9 mm round hole
                   outside: buck converter USB-C cable plugs in
                   inside: 2-wire pigtail → GPIO pin 4 (5V) and pin 6 (GND)
  Cable glands   — 2× PG7 metric gland on the same short wall
                   PG7 fits cables 3–6.5 mm diameter; M12 threaded body
                   thread gland from outside, tighten nut from inside
  BNO055 mount   — 2× M2 screws into the printed standoffs on the shelf
  Bottom mounts  — 4× M5 holes for standard pipe saddle clamps (hardware store)
  Servo clamp    — M6×50 hex bolt + nut; snug, not overtight (cracks plastic)
  Lidar arm      — M2.5×6 screws through horn holes; M3×8 screws for lidar

PRINT SETTINGS:
  Material: PETG (recommended) or ASA (better UV resistance for outdoor use)
  Layer height: 0.2 mm  |  Walls: 4 perimeters  |  Top/bottom: 5 layers
  Infill: 40% for bracket + lidar arm, 20% for enclosure walls and lid
  Orientation:
    enclosure_body  — open top face UP on print bed
    enclosure_lid   — flat top face DOWN on print bed (lip points UP while printing)
    servo_bracket   — clamp face down, arm extending up
    lidar_arm       — flat face down
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

# ── MG996R servo ─────────────────────────────────────────────────────────────
SERVO_FLANGE_SPAN = 47.5    # mounting hole C-C along flange
SERVO_HOLE_D      = 4.3     # M4 clearance

# ── Servo bracket ─────────────────────────────────────────────────────────────
TUBE_OD       = 38.1    # ← SET THIS to your mower frame tube outer diameter (mm)
                         #   1 inch = 25.4,  1.5 inch = 38.1,  2 inch = 50.8
CLAMP_WALL_T  = 5.0     # C-clamp wall thickness
ARM_LEN       = 70.0    # arm length: clamp centre → servo platform
ARM_W         = 28.0    # arm width
ARM_T         = 8.0     # arm thickness
PLATFORM_SIDE = 58.0    # servo mounting platform (square)
PLATFORM_T    = 5.0     # platform thickness
CLAMP_BOLT_D  = 6.5     # M6 clearance

# ── Lidar arm (TFmini-S ↔ servo horn adapter) ─────────────────────────────────
LIDAR_HOLE_SPACING = 31.0   # M3 hole C-C on TFmini-S
LIDAR_HOLE_D       = 3.2    # M3 clearance
HORN_HUB_D         = 6.3    # servo horn centre hub OD (+ 0.3 mm clearance)
HORN_SCREW_SPACING = 16.0   # horn outer screw C-C
HORN_SCREW_TAP     = 2.7    # M2.5 tap
ARM_PLATE_T        = 4.0    # plate thickness
ARM_PLATE_W        = 50.0   # plate width
ARM_PLATE_L        = 55.0   # plate length


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

# ── Bottom mounting holes ─────────────────────────────────────────────────────
# Four M5 clearance holes for pipe saddle clamp bolts.
# Saddle clamps (available at any hardware store) attach the enclosure to the frame tube.
for (mx, my) in [
    (-MOUNT_HOLE_X, -MOUNT_HOLE_Y),
    (-MOUNT_HOLE_X,  MOUNT_HOLE_Y),
    ( MOUNT_HOLE_X, -MOUNT_HOLE_Y),
    ( MOUNT_HOLE_X,  MOUNT_HOLE_Y),
]:
    enc = enc.cut(cyl_z(MOUNT_HOLE_D / 2, WALL + 1, mx, my, -0.5))

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
# PART 3: SERVO BRACKET
#
# Clamps to a vertical ROPS tube. A horizontal arm positions the servo so its
# output shaft points in the direction of travel (fore–aft axis of the mower).
# This makes the lidar sweep left–right across the mower path (cross-track scan).
#
# Origin: centre of tube bore, at midheight of clamp body.
# =============================================================================

print("Building servo bracket...")

clamp_r = TUBE_OD / 2 + CLAMP_WALL_T
clamp_h = TUBE_OD + 2 * CLAMP_WALL_T

# ── C-clamp body ──────────────────────────────────────────────────────────────
clamp = box_z(clamp_r, clamp_h, clamp_r * 2, z=-clamp_r)
# Bore for tube (+ 0.3 mm radial clearance for slip fit)
clamp = clamp.cut(cyl_z(TUBE_OD / 2 + 0.3, clamp_r * 2 + 2, z=-clamp_r - 1))
# Front slit so the clamp can flex open to accept the tube
slit_w = TUBE_OD * 0.60
clamp = clamp.cut(box_z(clamp_r + 1, slit_w, clamp_r * 2 + 2,
                         x=-(clamp_r + 1) / 2, z=-clamp_r - 1))

# Clamp bolt holes through the top and bottom ears (M6, axis along Y)
ear_x  = clamp_r / 2
for ez in [TUBE_OD / 2 + CLAMP_WALL_T / 2, -(TUBE_OD / 2 + CLAMP_WALL_T / 2)]:
    bolt = cq.Solid.makeCylinder(
        radius=CLAMP_BOLT_D / 2, height=clamp_h + 2,
        pnt=cq.Vector(-ear_x, -clamp_h / 2 - 1, ez),
        dir=cq.Vector(0, 1, 0),
    )
    clamp = clamp.cut(cq.Workplane("XY").add(bolt))

# ── Horizontal arm ────────────────────────────────────────────────────────────
arm = box_z(ARM_LEN, ARM_W, ARM_T, x=ARM_LEN / 2, z=-ARM_T / 2)
bracket = clamp.union(arm)

# ── Servo mounting platform at arm end ────────────────────────────────────────
plat_cx = ARM_LEN + PLATFORM_SIDE / 2 - CLAMP_WALL_T
platform = box_z(PLATFORM_SIDE, PLATFORM_SIDE, PLATFORM_T,
                 x=plat_cx, z=-ARM_T / 2 - PLATFORM_T)
# Servo flange holes (2× M4 along Y at SERVO_FLANGE_SPAN C-C)
for sy in [-SERVO_FLANGE_SPAN / 2, SERVO_FLANGE_SPAN / 2]:
    platform = platform.cut(
        cyl_z(SERVO_HOLE_D / 2, PLATFORM_T + 2,
              x=plat_cx, y=sy, z=-ARM_T / 2 - PLATFORM_T - 1)
    )
bracket = bracket.union(platform)

# ── Gusset triangle for arm rigidity ──────────────────────────────────────────
gusset = (
    cq.Workplane("XZ")
    .moveTo(ARM_LEN - 20, -ARM_T / 2)
    .lineTo(ARM_LEN + 5,  -ARM_T / 2)
    .lineTo(ARM_LEN + 5,  -ARM_T / 2 - PLATFORM_T - 18)
    .close()
    .extrude(ARM_W, both=True)
)
bracket = bracket.union(gusset)

print(f"  Bracket: arm {ARM_LEN:.0f} mm, clamp fits tube OD {TUBE_OD:.1f} mm")
print(f"  → Change TUBE_OD parameter to match your mower frame tube.")


# =============================================================================
# PART 4: LIDAR ARM
#
# Flat plate: one end attaches to the servo horn (hub + 2× M2.5 screw),
# other end holds the TFmini-S (2× M3 screw into 1.5 mm deep locating pocket).
# The lidar lens faces away from the plate when the arm is at nadir (0°).
#
# Origin: servo shaft axis at Y = 0 (pivot end), plate extends in +Y.
# =============================================================================

print("Building lidar arm...")

lidar_arm = box_z(ARM_PLATE_W, ARM_PLATE_L, ARM_PLATE_T)

# Servo horn hub clearance (centre of pivot end)
lidar_arm = lidar_arm.cut(
    cyl_z(HORN_HUB_D / 2, ARM_PLATE_T + 2, x=0, y=-ARM_PLATE_L / 2, z=-1)
)
# 2× M2.5 tap holes for horn screws
for hx in [-HORN_SCREW_SPACING / 2, HORN_SCREW_SPACING / 2]:
    lidar_arm = lidar_arm.cut(
        cyl_z(HORN_SCREW_TAP / 2, ARM_PLATE_T + 2, x=hx, y=-ARM_PLATE_L / 2, z=-1)
    )

# Lidar locating pocket (1.5 mm deep; prevents rotation)
lidar_cy = ARM_PLATE_L / 2 - 14.0
lidar_arm = lidar_arm.cut(
    box_z(32.0 + 0.5, 15.0 + 0.5, 1.5, y=lidar_cy, z=ARM_PLATE_T - 1.5)
)
# 2× M3 clearance holes for lidar mounting screws
for lx in [-LIDAR_HOLE_SPACING / 2, LIDAR_HOLE_SPACING / 2]:
    lidar_arm = lidar_arm.cut(
        cyl_z(LIDAR_HOLE_D / 2, ARM_PLATE_T + 2, x=lx, y=lidar_cy, z=-1)
    )

# Stiffening ribs on back face
for ry in [-ARM_PLATE_L * 0.15, ARM_PLATE_L * 0.15]:
    lidar_arm = lidar_arm.union(
        box_z(ARM_PLATE_W, 3.5, ARM_PLATE_T + 5.0, y=ry)
    )

try:
    lidar_arm = lidar_arm.edges("|Z").fillet(2.0)
except Exception:
    pass

print(f"  Lidar arm: {ARM_PLATE_W:.0f}W × {ARM_PLATE_L:.0f}L mm")


# =============================================================================
# COMMAND-LINE EXPORT
# =============================================================================

if __name__ == "__main__":
    out = os.path.dirname(os.path.abspath(__file__))
    for name, part in [("enclosure_body", enc), ("enclosure_lid", lid),
                        ("servo_bracket", bracket), ("lidar_arm", lidar_arm)]:
        path = os.path.join(out, f"{name}.stl")
        cq.exporters.export(part, path)
        print(f"Exported: {path}")


# =============================================================================
# CQ-EDITOR — edit show_object() calls to preview different parts.
# Show body + lid together by default (position lid above body for clarity).
# =============================================================================

show_object(enc,
            name="enclosure_body",
            options={"color": "#3a6ebf", "alpha": 0.85})

show_object(lid.translate((0, 0, EXT_H + FLANGE_T + 5)),  # float lid above body
            name="enclosure_lid",
            options={"color": "#5aaa6a", "alpha": 0.85})

# Uncomment to preview other parts:
# show_object(bracket,   name="servo_bracket",  options={"color": "#bf8c3a"})
# show_object(lidar_arm, name="lidar_arm",       options={"color": "#bf3a6e"})
