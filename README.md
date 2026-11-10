# LSSU Ri3D swerve template

Our robot code base for BIOCORE: swerve on 4 SDS MK5n (R1), NEO v1.1 drive on SPARK MAX, Kraken X44
steer, CANcoders, NavX MXP (or a Pigeon 1.0), 30x30 frame. Everything is ready before kickoff, so
build day is just adding mechanisms.

Read [STANDARDS.md](STANDARDS.md) before you write anything, it's short.

## Setup

1. Install [WPILib 2026](https://github.com/wpilibsuite/allwpilib/releases) (comes with VS Code).
2. Clone this repo and open the folder in WPILib VS Code:

   ```bash
   git clone https://github.com/LSSU-Ri3D/swerve-template.git
   ```
3. `./gradlew build`. The first one downloads everything and takes a few minutes.
4. Set your team number in `.wpilib/wpilib_preferences.json` to whatever the roboRIO was imaged
   with (it's `0` right now). Deploying over USB works whatever the number is.

Simulate: `WPILib: Simulate Robot Code` in VS Code. Field-relative drive, autos and odometry all
work in sim.

## Driver controls (Xbox, port 0)

| Input | Does |
|---|---|
| Left stick | drive, field relative |
| Right stick X | turn |
| LB (hold) | slow mode |
| RB (hold) | robot relative |
| X (hold) | X-lock |
| Start | zero heading, point the robot away from you first |

## Swapping the gyro

`DriveConstants.GYRO`. `NAVX_MXP` for the NavX in the MXP port, `PIGEON_1` for a Pigeon 1.0 on CAN
ID 2 (change it in `Ports.PIGEON_1`). That's the only line.

## Hardware notes

- **MK5n R1:** 7.03:1 drive (12:54, 32:25, 15:30), 287:11 steer, 4" wheel, ~14.1 ft/s free with a
  NEO. For R2/R3 change the pinion in `DRIVE_GEAR_RATIO` **and** `COUPLE_RATIO`.
- **Geometry:** wheel centers are 24.757" apart both ways, measured off the Robot 27 chassis in
  Onshape.
- **Steering** closes the loop on the Talon with the CANcoder as a remote sensor (no Phoenix Pro
  needed).
