package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

// Every auto. PathPlanner autos show up in the chooser by themselves once they're saved
// in src/main/deploy/pathplanner, code-only autos get added in buildChooser.
public final class Autos {
  private Autos() {}

  // Anything a PathPlanner auto calls by name. The name here has to match the GUI exactly, and if
  // it doesn't PathPlanner just skips it, so copy paste it
  public static void registerNamedCommands(Drive drive) {
    NamedCommands.registerCommand("stop", drive.stop());
  }

  public static SendableChooser<Command> buildChooser(Drive drive) {
    SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
    chooser.addOption("Leave (no path, just drive forward)", leave(drive));
    return chooser;
  }

  // Drives forward ~1.5 m robot relative. Works even if PathPlanner, the gyro or odometry is
  // broken, so it's the backup if nothing else works
  public static Command leave(Drive drive) {
    return drive
        .driveRobotRelative(new ChassisSpeeds(1.0, 0, 0))
        .withTimeout(1.5)
        .andThen(drive.stop())
        .withName("Autos.leave");
  }
}
