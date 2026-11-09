package frc.robot;

// Every CAN ID and controller port on the robot 
// IDs are unique across ALL device types (REV and CTRE would let them overlap, we don't), so
// "23" always means one physical thing. Tens digit is the group:
//   1-9    power and robot-wide stuff
//   1x-4x  swerve corners FL, FR, BL, BR -> x1 drive NEO, x2 steer Kraken, x3 CANcoder
//   5x+    one decade per mechanism
//
// Put the ID on a label maker sticker on the motor too (really).
public final class Ports {
  private Ports() {}

  // Driver Station USB order
  public static final int DRIVER_CONTROLLER = 0;
  public static final int OPERATOR_CONTROLLER = 1;

  // WPILib expects a REV PDH on 1. The NavX goes in the MXP port, not CAN, so it doesn't get an
  // ID, the Pigeon 1.0 only matters if DriveConstants.GYRO is set to it
  public static final int PDH = 1;
  public static final int PIGEON_1 = 2;

  public static final int FL_DRIVE = 11;
  public static final int FL_STEER = 12;
  public static final int FL_ENCODER = 13;

  public static final int FR_DRIVE = 21;
  public static final int FR_STEER = 22;
  public static final int FR_ENCODER = 23;

  public static final int BL_DRIVE = 31;
  public static final int BL_STEER = 32;
  public static final int BL_ENCODER = 33;

  public static final int BR_DRIVE = 41;
  public static final int BR_STEER = 42;
  public static final int BR_ENCODER = 43;

  // Mechanisms. Rename these once the real ones exist
  public static final int EXAMPLE_ROLLER = 51;
  public static final int EXAMPLE_PIVOT = 61;
}
