package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

// Stick shaping. Lives here and not in Drive because how the sticks feel is a driver thing, not a
// drivetrain thing.
public final class DriverInput {
  private DriverInput() {}

  public static final double DEADBAND = 0.08;

  // Deadband on the whole stick (so diagonals don't get a cross-shaped dead zone) and square it so
  // small moves are gentle. Comes out as a fraction of top speed, never longer than 1
  public static Translation2d translation(double forward, double left) {
    var stick = new Translation2d(forward, left);
    double magnitude = MathUtil.applyDeadband(Math.min(stick.getNorm(), 1.0), DEADBAND);
    if (magnitude == 0) {
      return Translation2d.kZero;
    }
    return new Translation2d(magnitude * magnitude, stick.getAngle());
  }

  public static double rotation(double ccw) {
    double value = MathUtil.applyDeadband(ccw, DEADBAND);
    return Math.copySign(value * value, value);
  }
}
