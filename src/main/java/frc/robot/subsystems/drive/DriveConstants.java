package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Ports;

// Our drivebase: 4x SDS MK5n on the R1 (low) ratio, NEO v1.1 drive on SPARK MAX, Kraken X44 steer,
// CANcoders, 4" wheels, 30x30 frame. Anything tagged BRING-UP gets set on the real robot by going
// through the checklist in the README, don't guess them.
public final class DriveConstants {
  private DriveConstants() {}

  public enum GyroType {
    NAVX_MXP,
    PIGEON_1
  }

  // Which gyro is on the robot right now. This is the only line to change to swap them
  public static final GyroType GYRO = GyroType.NAVX_MXP;

  public record ModuleSpec(
      String name,
      int driveId,
      int steerId,
      int encoderId,
      double encoderOffsetRotations,
      Translation2d location) {}

  // ---- Geometry, from the Robot 27 chassis in Onshape ----

  // Wheel centers are 12.379" off center both ways (1.5*sqrt(2)" in from the rail centerline), so
  // it's a square. If the frame changes, re-measure from CAD
  public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(24.757);
  public static final double WHEEL_BASE_METERS = Units.inchesToMeters(24.757);

  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

  public static final double DRIVE_BASE_RADIUS_METERS =
      Math.hypot(TRACK_WIDTH_METERS / 2, WHEEL_BASE_METERS / 2);

  // ---- Gearing, off SDS's MK5n spec sheet ----

  // R1 is 12:54, 32:25, 15:30 = 7.03:1. For R2 swap the pinion to 14T, R3 is 16T, and change it
  // in COUPLE_RATIO too
  public static final double DRIVE_GEAR_RATIO = (54.0 / 12.0) * (25.0 / 32.0) * (30.0 / 15.0);

  public static final double STEER_GEAR_RATIO = 287.0 / 11.0;

  // The 54T first stage gear sits on the steering axis, so just turning a module spins the wheel
  // a bit even with the drive motor stopped (54 / pinion teeth motor turns per module turn). If we
  // ignore it odometry drifts ~0.2 m every time a module spins all the way around.
  // BRING-UP: the sign gets checked with the coupling test in the README
  public static final double COUPLE_RATIO = 54.0 / 12.0;

  public static final double DRIVE_METERS_PER_MOTOR_ROTATION =
      2 * Math.PI * WHEEL_RADIUS_METERS / DRIVE_GEAR_RATIO;

  // ---- Speed ----

  public static final DCMotor DRIVE_MOTOR = DCMotor.getNEO(1);

  // Theoretical free speed, ~4.29 m/s (14.1 ft/s). On carpet we'll really get ~85% of that
  public static final double MAX_SPEED_MPS =
      DRIVE_MOTOR.freeSpeedRadPerSec / DRIVE_GEAR_RATIO * WHEEL_RADIUS_METERS;

  public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC =
      MAX_SPEED_MPS / DRIVE_BASE_RADIUS_METERS;

  // ---- Directions (BRING-UP) ----

  // flip if pushing forward spins every wheel backwards
  public static final boolean DRIVE_INVERTED = false;

  // flip if a module shakes or spins away instead of holding its angle
  public static final boolean STEER_INVERTED = false;

  // flip if turning a module CCW (looking down at it) makes its angle go down
  public static final boolean ENCODER_CLOCKWISE_POSITIVE = false;

  // ---- Current limits ----

  public static final int DRIVE_CURRENT_LIMIT_AMPS = 50;
  public static final double STEER_STATOR_LIMIT_AMPS = 40;
  public static final double STEER_SUPPLY_LIMIT_AMPS = 30;

  // ---- Gains ----

  // volts per m/s. This is just 12V / free speed, swap in real SysId numbers if we get time
  public static final double DRIVE_KV = 12.0 / MAX_SPEED_MPS;
  public static final double DRIVE_KS = 0.15;

  // runs on the SPARK MAX, so it's duty cycle per m/s of error
  public static final double DRIVE_KP = 0.04;

  // runs on the Talon, volts per rotation of module angle error
  public static final double STEER_KP = 60.0;
  public static final double STEER_KD = 0.5;

  // path following, m/s per meter of error and rad/s per radian of error
  public static final double PATH_TRANSLATION_KP = 5.0;
  public static final double PATH_ROTATION_KP = 5.0;

  // ---- For PathPlanner. Weigh the robot with bumpers + battery and fix MASS ----

  public static final double ROBOT_MASS_KG = Units.lbsToKilograms(115);
  public static final double ROBOT_MOI_KG_M2 = 6.0;
  public static final double WHEEL_COF = 1.2;

  // ---- Modules (BRING-UP: encoder offsets) ----

  public static final ModuleSpec FRONT_LEFT =
      new ModuleSpec(
          "FrontLeft",
          Ports.FL_DRIVE,
          Ports.FL_STEER,
          Ports.FL_ENCODER,
          0.0,
          new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2));

  public static final ModuleSpec FRONT_RIGHT =
      new ModuleSpec(
          "FrontRight",
          Ports.FR_DRIVE,
          Ports.FR_STEER,
          Ports.FR_ENCODER,
          0.0,
          new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

  public static final ModuleSpec BACK_LEFT =
      new ModuleSpec(
          "BackLeft",
          Ports.BL_DRIVE,
          Ports.BL_STEER,
          Ports.BL_ENCODER,
          0.0,
          new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2));

  public static final ModuleSpec BACK_RIGHT =
      new ModuleSpec(
          "BackRight",
          Ports.BR_DRIVE,
          Ports.BR_STEER,
          Ports.BR_ENCODER,
          0.0,
          new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

  // always FL, FR, BL, BR. Every array of module stuff in this package is in this order
  public static final ModuleSpec[] MODULES = {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          FRONT_LEFT.location(),
          FRONT_RIGHT.location(),
          BACK_LEFT.location(),
          BACK_RIGHT.location());
}
