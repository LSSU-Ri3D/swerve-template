package frc.robot.subsystems.example;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Ports;
import frc.robot.util.Hardware;

// Example of a mechanism that goes to positions (arm, wrist, pivot, an elevator is the same idea
// with Elevator_Static gravity and meters). Same deal as ExampleRoller, copy it, don't use it.
//
// Position commands hold forever and never finish on their own. If something has to wait for the
// arm to get there, wait on atGoal, don't make goTo() end, because the second it ends the default
// command takes over and the arm drops back to stow.
@Logged(strategy = Strategy.OPT_IN)
public class ExamplePivot extends SubsystemBase {
  // Named positions, all in degrees with 0 = arm straight out horizontal (that's what kG with
  // Arm_Cosine expects)
  public enum Setpoint {
    STOW(90),
    INTAKE(-10),
    SCORE(45);

    final double degrees;

    Setpoint(double degrees) {
      this.degrees = degrees;
    }
  }

  // motor rotations per arm rotation
  private static final double GEAR_RATIO = 60.0;
  private static final double TOLERANCE_DEGREES = 2.0;
  private static final double MIN_DEGREES = -15;
  private static final double MAX_DEGREES = 95;

  private final TalonFX motor = new TalonFX(Ports.EXAMPLE_PIVOT);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final StatusSignal<Angle> position = motor.getPosition();

  private Setpoint goal = Setpoint.STOW;

  public final Trigger atGoal =
      new Trigger(() -> Math.abs(getAngleDegrees() - goal.degrees) < TOLERANCE_DEGREES);

  public ExamplePivot() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    // everything below is in arm rotations, not motor rotations, thanks to this
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kG = 0.3;
    config.Slot0.kS = 0.1;
    config.Slot0.kV = 7.0;
    config.Slot0.kP = 40;
    config.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    config.MotionMagic.MotionMagicAcceleration = 2.0;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(MAX_DEGREES);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(MIN_DEGREES);
    Hardware.configure("ExamplePivot", () -> motor.getConfigurator().apply(config));

    // No absolute encoder, so the arm HAS to be sitting at stow (on its hard stop) when the robot
    // turns on. Put that on the pre-match checklist
    motor.setPosition(Units.degreesToRotations(Setpoint.STOW.degrees));
  }

  public Command goTo(Setpoint setpoint) {
    return run(() -> {
          goal = setpoint;
          motor.setControl(
              positionRequest.withPosition(Units.degreesToRotations(setpoint.degrees)));
        })
        .withName("ExamplePivot.goTo(" + setpoint + ")");
  }

  // for when it didn't start on the hard stop. Push it there by hand and hit this
  public Command zeroAtStow() {
    return runOnce(() -> motor.setPosition(Units.degreesToRotations(Setpoint.STOW.degrees)))
        .ignoringDisable(true)
        .withName("ExamplePivot.zeroAtStow");
  }

  @Logged
  public double getAngleDegrees() {
    return Units.rotationsToDegrees(position.refresh().getValueAsDouble());
  }

  @Logged
  public double getGoalDegrees() {
    return goal.degrees;
  }
}
