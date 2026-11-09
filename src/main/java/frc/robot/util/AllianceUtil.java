package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class AllianceUtil {
  private AllianceUtil() {}

  // Blue until the FMS / DS says otherwise, so sim and the practice field act like blue
  public static boolean isRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }
}
