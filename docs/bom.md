# Bill of materials

Everything needed for one hand: **≈ $460**, of which the camera is $272 and the
two servos are $100. The printed parts come to under $10 of filament.

Prices are the ones quoted in the paper (2024, USD). The fastener rows are
**pack** prices — an assortment of M3 screws costs $12.49 whether you need 32 of
them or 4 — so a lab that already owns metric hardware builds this for closer to
$400.

## Printed

Print these yourself from [`hand/stls/`](../hand/stls); see the
[build guide](build.md#1-print-the-parts) for orientation and fit.

| Part | Qty | Material cost |
| --- | ---: | ---: |
| Top base | 1 | $1.51 |
| Top base cover | 1 | $2.05 |
| Bottom base | 1 | $1.17 |
| Bottom base cover | 1 | $1.15 |
| Servo crank | 2 | $0.47 |
| Servo coupler | 4 | $1.56 |
| Servo rocker | 2 | $0.28 |
| Finger | 2 | $0.86 |
| | | **$9.05** |

≈ 181 cm³ of PLA in total — about 220 g printed solid, roughly half that at a
normal infill.

## Bought

| Item | Qty | Cost | Where |
| --- | ---: | ---: | --- |
| Dynamixel **AX-12A** servo | 2 | $99.80 | [robotis.us](https://www.robotis.us/dynamixel-ax-12a/) |
| **OpenRB-150** controller board | 1 | $24.90 | [robotis.us](https://www.robotis.us/openrb-150/) |
| **Intel RealSense D405** depth camera | 1 | $272.00 | [store.intelrealsense.com](https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d405.html) |
| Ball bearings, 5 × 10 × 4 mm (MR105-2RS) | 8 | $9.99 | [amazon](https://www.amazon.com/dp/B082PS3QDJ) |
| M3 × 8 mm round aluminium standoff | 8 | $9.99 | [amazon](https://www.amazon.com/dp/B08HL6SCDG) |
| M3 × 6 mm round aluminium standoff | 4 | $9.99 | [amazon](https://www.amazon.com/dp/B08HL74BBD) |
| M2.5 × 10 mm standoff | 4 | $9.99 | [amazon](https://www.amazon.com/dp/B0B7SNCFF1) |
| M3 nuts and bolts | 32 | $12.49 | [amazon](https://www.amazon.com/dp/B0BJVN748D) |
| M2 nuts and bolts | 8 | $9.99 | [amazon](https://www.amazon.com/dp/B07DVLLKZ3) |
| Electrical wire | 2 | $0.45 | any |
| | | **$459.59** | |

The bearings are what make the linkage worth building: pressed into the PLA they
take out most of the friction and, because the seat is an interference fit, most
of the play as well.

## Also needed, not counted above

- A **12 V supply, 3 A or better** (36 W) for the servo board.
- A **USB 3** port and cable for the camera; USB 2 will not carry its depth stream.
- A robot arm, if the hand is to go anywhere. The stack ships an interface for
  **Universal Robots** (UR5/UR10) over RTDE; anything else means writing the
  equivalent of [`magpie/ur5.py`](../src/magpie/ur5.py).
- [**Dynamixel Wizard 2.0**](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
  — free, and the only practical way to set servo IDs and read back the angles
  the [calibration step](build.md#6-calibrate-the-fingers) needs.

## Substitutions worth knowing about

- **The camera** is the single biggest line item and the one place a cheaper part
  changes the design: the D405's 7–50 cm range is what makes palm mounting work.
  A camera with a longer minimum range sees nothing at the moment it matters.
- **The servos** are chosen for their digital interface, not their strength. Force
  control here *is* the AX-12A's current limit; a hobby servo with a PWM input
  cannot do any of this, whatever its torque rating.
