package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Ports;

// The gyro, whichever one is actually on the robot. Swap it with DriveConstants.GYRO, nothing
// else in the code cares which one it is. Yaw is CCW positive like the rest of WPILib.
interface Gyro {
  Rotation2d getYaw();

  boolean isConnected();

  static Gyro create(DriveConstants.GyroType type) {
    return switch (type) {
      case NAVX_MXP -> new NavX();
      case PIGEON_1 -> new Pigeon1();
    };
  }

  // NavX plugged into the MXP port on the roboRIO. It calibrates for a few seconds after boot, so
  // don't touch the robot while it's turning on. If it's mounted on its side (not flat on the RIO)
  // it needs an omnimount calibration first, see Studica's docs
  final class NavX implements Gyro {
    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

    @Override
    public Rotation2d getYaw() {
      return navx.getRotation2d();
    }

    @Override
    public boolean isConnected() {
      return navx.isConnected();
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
