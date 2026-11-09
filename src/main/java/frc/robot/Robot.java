package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Don't put robot logic in here. Robot just runs the scheduler and hands off to RobotContainer,
// if you're about to add something to a xxxPeriodic() it almost certainly belongs in a command.
@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {
  @Logged private final RobotContainer container;
  private Command autonomousCommand;

  public Robot() {
    // Logs every NetworkTables value plus DS/joystick data to a .wpilog on the RIO (or a USB stick
    // if one's plugged in). Pull them with the DataLog Tool or open them in AdvantageScope
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Elastic's "Download layout from robot" pulls src/main/deploy/elastic-layout.json from here
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    container = new RobotContainer();
    Epilogue.bind(this);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = container.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
