package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.studica.frc.Navx;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Ports;

// The gyro, whichever one is actually on the robot. Swap it with DriveConstants.GYRO, nothing
// else in the code cares which one it is. Yaw is CCW positive like the rest of WPILib.
interface Gyro {
  Rotation2d getYaw();

  boolean isConnected();

  static Gyro create(DriveConstants.GyroType type) {
    return switch (type) {
      case NAVX3 -> new NavX3();
      case PIGEON_1 -> new Pigeon1();
    };
  }

  // NavX3 on CAN (StudicaLib). Same deal as the old NavX, it calibrates for a few seconds after
  // boot
  // so don't touch the robot while it's turning on. Its yaw is CCW positive already (the old AHRS
  // class flipped it for us, this one doesn't need to), but spin the robot CCW on blocks once and
  // make sure the heading goes up before trusting it
  final class NavX3 implements Gyro {
    private final Navx navx = new Navx(Ports.NAVX3);

    @Override
    public Rotation2d getYaw() {
      return navx.getRotation2d();
    }

    // StudicaLib has no isConnected(), it hands back a yaw of 360 when it isn't hearing from the
    // NavX3 (real yaw is -180 to 180)
    @Override
    public boolean isConnected() {
      return Math.abs(navx.getYaw().in(Degrees)) < 360;
    }
  }

  // Pigeon 1.0 on CAN (Phoenix 5, Phoenix 6 doesn't know about it)
  final class Pigeon1 implements Gyro {
    private final PigeonIMU pigeon = new PigeonIMU(Ports.PIGEON_1);

    Pigeon1() {
      pigeon.configFactoryDefault();
    }

    @Override
    public Rotation2d getYaw() {
      return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    @Override
    public boolean isConnected() {
      return pigeon.getState() == PigeonIMU.PigeonState.Ready;
    }
  }
}
