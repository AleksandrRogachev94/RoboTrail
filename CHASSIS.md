# Chassis Design & 3D Printing Guide

Two-tier platform with tank treads for Raspberry Pi 5 SLAM robot. Design in **Fusion 360**.

```
┌─────────────────────────────┐
│   TOP PLATE                 │  ← Pi 5, sensors, OLED
├─────────────────────────────┤  ← 35mm standoffs (printed)
│   BOTTOM PLATE              │  ← Battery, motors, buck converter
└─────────────────────────────┘
        ◯═══════════◯  ← Tank treads
```

## Dimensions

| Dimension            | Value          | Notes                              |
| -------------------- | -------------- | ---------------------------------- |
| **Chassis length**   | 140mm          | Fits 2S LiPo battery lengthwise    |
| **Chassis width**    | 100mm          | Stable platform for components     |
| **Plate thickness**  | 3-4mm          | Rigid but lightweight              |
| **Standoff height**  | 35-40mm        | Clearance for battery + wiring     |
| **Wheel diameter**   | 32.5mm         | ~2.5 cm/sec with 28BYJ-48 steppers |
| **Wheelbase**        | ~90mm          | Motor/idler center to center       |
| **Motor from edge**  | ~25mm          | Required for 45mm mounting wall    |
| **Track links**      | 15-17 per side | 30-34 total (std SMARS: 32)        |
| **Track link pitch** | 17.4mm         | Length per link (from STL)         |

## Component Dimensions (for Fusion 360 CAD)

### 28BYJ-48 Stepper Motor

```
        ┌───┐ ← Shaft: 5mm with flats, 6mm tall
    ┌───┴───┴───┐
    │  ○     ○  │ ← Mounting holes: 35mm apart, M2 threads
    │    28mm   │ ← Body diameter
    └───────────┘
    Height: 19mm (body only)
    Wiring: 5-pin connector, exits from side
```

### SG90 Micro Servo

```
    ┌─────────────────┐
    │  ┌───┐          │ ← Shaft: 4.8mm, centered 5mm from edge
    ├──┤   ├──────────┤ ← Mounting tabs: 32mm total, 2mm holes
    │  └───┘          │
    └─────────────────┘
    Body: 23mm × 12mm × 22mm
    Tab thickness: 2.5mm each side
```

### Raspberry Pi 5

```
    Mounting holes: 58mm × 49mm (M2.5 standard, but M2 works)
    Board: 85mm × 56mm
    Height with heatsink: ~15mm
```

### Integrated Standoffs (Bottom Plate)

Print standoffs as part of the bottom plate—no separate pieces needed:

```
    Side view (cross-section):

            Top plate (3mm)
    ════════╪════════╪════════  ← Screws down into standoffs
            ║        ║
            ║  35mm  ║  ← Integrated standoffs
            ║        ║
    ┌───────╨────────╨───────┐
    │     Bottom plate       │  (4mm thick)
    └────────────────────────┘
```

**Standoff positions:**

- **Front standoffs:** 5mm from front edge (no motor interference)
- **Rear standoffs:** 30mm from rear edge (clears motor mounting zone)

In Fusion 360: Extrude 8mm diameter cylinders (35mm tall) from corner positions. Use 5mm inset for front corners, 30mm inset for rear corners. Add M2 screw holes (2.2mm) at the top.

### Pi Mounting Bosses (Top Plate)

The Pi needs ~5mm clearance from the plate for airflow and to avoid shorts. Print small bosses integrated into the top plate:

```
    Top plate cross-section:

    ┌────╥────────────────╥────┐
    │    ║ 5mm            ║    │  ← Bosses
    │    ╨                ╨    │
    │        [  Pi 5  ]        │  ← Board sits on bosses
    └──────────────────────────┘

    Boss: 5mm tall, 6mm diameter, M2 hole (2.2mm) centered
    Pattern: 58mm × 49mm (matches Pi mounting holes)
```

### Corner Walls for Wheels

Add small vertical walls at all 4 corners to mount drive wheels and idlers:

```
┌─────────────────────────────────┐
│▓▓│                          │▓▓│ ← Idler mounts (front)
│  │                          │  │
│  │      [flat plate]        │  │
│  │                          │  │
│▓▓│                          │▓▓│ ← Motor mounts (rear)
└─────────────────────────────────┘

Wall dimensions:
- Height: ~45mm (to fit 35mm motor mounting hole spacing + margins)
- Thickness: 2-3mm
- Position: Centered ~25mm from front/rear edges
```

**Tip:** Import SMARS corner brackets from official STL to ensure wheel alignment.

## Parts List

### All 3D Printed

| Part                        | Quantity | Notes                                        |
| --------------------------- | -------- | -------------------------------------------- |
| Top plate (140×100×3mm)     | 1×       | With Pi bosses, servo slot, sensor holes     |
| Bottom plate with standoffs | 1×       | 4mm plate + 35mm integrated corner standoffs |
| Modified wheel (5mm shaft)  | 4×       | Search "SMARS 28BYJ-48 wheel"                |
| Track links                 | 40-44×   | Print extras for tension adjustment          |

> ⚠️ **Important:** Standard SMARS wheels are for DC motors (3mm D-shaft). The 28BYJ-48 stepper has a 5mm shaft with flats. Use modified wheels or edit the STL in Fusion 360.

### Hardware (Buy)

| Part            | Quantity | Notes                   |
| --------------- | -------- | ----------------------- |
| M2 × 6mm screws | 16×      | For Pi, servo, plates   |
| M2 × 8mm screws | 8×       | For standoffs to plates |
| M2 nuts         | 16×      | Or use heat-set inserts |

> **M2 screw length:** 8mm works for most applications. Avoid 5mm—too short for some mounts.

## Print Settings

| Setting          | Value       | Notes                         |
| ---------------- | ----------- | ----------------------------- |
| **Material**     | PLA or PETG | PLA easier, PETG more durable |
| **Layer height** | 0.2mm       | Speed/quality balance         |
| **Infill**       | 20-30%      | Plates need rigidity          |
| **Walls**        | 3           | Strong mounting holes         |

**Standoffs:** Print vertically, 40-50% infill for strength.

**Track links:** Print in PLA at 100% infill (or TPU if available). Connect with 1.75mm filament scraps as pins.

## Layout

### Motor Configuration

**Use rear drive** (both motors at rear corners):

- Simpler wiring—both ULN2003 drivers together
- Symmetric and easier to debug
- Better for 140mm chassis (diagonal drive was for compact SMARS)

### Bottom Plate

Motors mount at **rear** corners. Battery runs lengthwise in the center.

```
                  FRONT
    ┌─────────────────────────────────┐
    │  ○                           ○  │  ← Standoffs (front)
    │                                 │
    │   [INA219]   [BUCK CONVERTER]   │  ← Power monitoring + regulation
    │                                 │
    │         ┌─────────────┐         │
    │         │   BATTERY   │         │  ← Battery: 130mm long
    │         │  (2S LiPo)  │         │     centered, runs front-back
    │         └─────────────┘         │
    │                                 │
    │  ┌─────┐             ┌─────┐   │
    │  │MOTOR│             │MOTOR│   │  ← Motors (rear)
    │  │ L   │             │  R  │   │
    │  └─────┘             └─────┘   │
    │  ○                           ○  │  ← Standoffs (rear)
    └─────────────────────────────────┘
                  REAR
```

### Battery Mounting

Use a 4-wall pocket to hold the battery in place:

```
Top view of battery pocket:
┌──────────────────────────────────────┐
│                                      │
│      ┌───────────────────┐           │
│      │                   │← Wall     │
│   ┌──┤                   ├──┐        │
│   │  │     BATTERY       │  │        │
│   │  │                   │  │        │
│   └──┤   ═══════════     ├──┘        │
│      │       ↑           │           │
│      └───────┼───────────┘           │
│              │                       │
│         Wire slot                    │
└──────────────────────────────────────┘

Wall height: ~15mm (just above battery thickness)
Wall thickness: 2-3mm
Wire slot: 10mm wide gap in one wall
```

### Top Plate

Servo+ToF mounts at **front center** for unobstructed 180° scanning. Pi is centered with sensors around it.

```
                  FRONT
    ┌─────────────────────────────────┐
    │  ○        [SERVO+ToF]        ○  │  ← Servo CENTER-FRONT
    │              ↑                  │     (not corner!)
    │         Scanning arc            │
    │                                 │
    │      ┌───────────────┐          │
    │      │  Raspberry    │  [OLED]  │
    │      │     Pi 5      │          │
    │      └───────────────┘          │
    │                        [IMU]    │
    │  ○                           ○  │
    └─────────────────────────────────┘
                  REAR
```

## Fusion 360 Design Tips

1. **Start with sketches**: Create top and bottom plate outlines as 2D sketches before extruding
2. **Use components**: Make each part (plate, standoff, servo mount) a separate component
3. **Parametric design**: Use parameters for key dimensions (plate size, standoff height, screw size) so you can adjust easily
4. **Check clearances**: Use section analysis to verify standoff height provides enough battery clearance
5. **Import SMARS parts**: Import the SMARS chassis STL, extract corner brackets for motor/idler mounts
6. **Export as STL**: Right-click component → Save As Mesh → STL format for slicing

### Recommended Parameters

| Parameter      | Value | Description                |
| -------------- | ----- | -------------------------- |
| `plate_length` | 140   | Overall chassis length     |
| `plate_width`  | 100   | Overall chassis width      |
| `standoff_h`   | 35    | Standoff height            |
| `screw_dia`    | 2.0   | M2 screw hole diameter     |
| `screw_clear`  | 2.2   | M2 clearance hole diameter |
| `front_inset`  | 5     | Front standoff inset       |
| `rear_inset`   | 30    | Rear standoff inset        |
| `motor_inset`  | 25    | Motor center from edge     |

## Checklist Before Printing

1. Print all parts and assemble track chains (18-19 links per side)
2. Mount motors to bottom plate rear using SMARS motor mount brackets
3. Press-fit wheels onto 5mm motor shafts
4. Attach idler wheels at front corners using SMARS idler mounts
5. Loop tracks around drive and idler wheels
6. Insert M2 screws through standoffs and attach to bottom plate (or let self-tap)
7. Mount battery in pocket on bottom plate
8. Mount buck converter on bottom plate
9. Attach top plate to standoffs
10. Mount Pi 5 on bosses, servo (front center), and sensors
11. Wire everything (I2C bus, motor drivers, power)

## Checklist Before Printing

- [ ] Servo mount is centered at front (not corner)
- [ ] Motors are at rear, not front
- [ ] Pi 5 mounting holes match 58×49mm pattern
- [ ] Pi bosses are 5mm tall
- [ ] Standoffs are integrated into bottom plate (35mm tall)
- [ ] Plate holes match component mounting patterns
- [ ] Wheel hub fits 5mm shaft with flats (not 3mm D-shaft)
- [ ] Battery pocket has 4 walls with wire slot
- [ ] Corner walls added for motor and idler mounts

## Resources

- [SMARS Official](https://www.smarsfan.com/) — Original design files
- [Thingiverse SMARS](https://www.thingiverse.com/thing:2662828) — STL downloads
- Search: "SMARS 28BYJ-48" for stepper-compatible wheels
