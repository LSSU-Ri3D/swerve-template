package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DriverInput;
import java.util.function.Supplier;

// Where the robot gets put together: every subsystem gets made here, every button gets bound here,
// every default command gets set here. If you're looking for "what does this button do", it's in
// this file
@Logged(strategy = Strategy.OPT_IN)
public class RobotContainer {
  // fraction of top speed while the driver holds left bumper
  private static final double SLOW_MODE_SCALE = 0.35;

  private final CommandXboxController driver = new CommandXboxController(Ports.DRIVER_CONTROLLER);

  @SuppressWarnings("unused") // nothing's bound to it until we have mechanisms
  private final CommandXboxController operator =
      new CommandXboxController(Ports.OPERATOR_CONTROLLER);

  @Logged private final Drive drive = new Drive();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // named commands have to exist before PathPlanner loads any autos, so this goes first
    Autos.registerNamedCommands(drive);
    autoChooser = Autos.buildChooser(drive);
    SmartDashboard.putData("Auto", autoChooser);
    // bring-up buttons, click them in Elastic (or some other dashboard I just like elastic)
    SmartDashboard.putData("Bring-up/Coupling test", drive.couplingTest());

    configureDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(drive.joystickDrive(driverTranslation(), this::driverRotation, true));
  }

  // Driver, Xbox on port 0:
  //   left stick     drive (field relative)
  //   right stick X  turn
  //   LB (hold)      slow mode
  //   RB (hold)      robot relative
  //   X (hold)       X-lock the wheels
  //   Start          zero heading, point the robot away from you first
  private void configureDriverBindings() {
    driver
        .rightBumper()
        .whileTrue(drive.joystickDrive(driverTranslation(), this::driverRotation, false));
    driver.x().whileTrue(drive.xLock());
    driver.start().onTrue(drive.zeroHeading());
  }

  // Operator, Xbox (or something else) on port 1. Mechanism buttons go here
  private void configureOperatorBindings() {}

  // Xbox sticks are +Y down and +X right, robot is +X forward and +Y left, hence the minus signs
  private Supplier<Translation2d> driverTranslation() {
    return () ->
        DriverInput.translation(-driver.getLeftY(), -driver.getLeftX()).times(speedScale());
  }

  private double driverRotation() {
    return DriverInput.rotation(-driver.getRightX()) * speedScale();
  }

  private double speedScale() {
    return driver.leftBumper().getAsBoolean() ? SLOW_MODE_SCALE : 1.0;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
