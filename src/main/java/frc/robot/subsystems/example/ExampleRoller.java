package frc.robot.subsystems.example;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Ports;
import frc.robot.util.Hardware;

// Example of what a subsystem looks like (see STANDARDS.md). Copy this folder for a real intake /
// indexer / anything that just spins, rename it, then delete the example. It isn't created in
// RobotContainer, it's only here so it keeps compiling.
//
// The shape: motors are private, the only way to make it move is a command from one of the
// factory methods below, and anything other code needs to know about it is a Trigger or a getter.
@Logged(strategy = Strategy.OPT_IN)
public class ExampleRoller extends SubsystemBase {
  // constants for one subsystem live at the top of that subsystem. They only get their own
  // XxxConstants file once there's too many to read at a glance (like drive)
  private static final double INTAKE_VOLTS = 8.0;
  private static final double EJECT_VOLTS = -6.0;
  private static final double STALL_AMPS = 30.0;
  private static final double STATOR_LIMIT_AMPS = 40.0;

  private final TalonFX motor = new TalonFX(Ports.EXAMPLE_ROLLER);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final StatusSignal<Current> statorCurrent = motor.getStatorCurrent();

  // True once a game piece is jammed in and the roller stalls on it. Debounced so the current
  // spike when the motor first starts doesn't count
  public final Trigger hasPiece =
      new Trigger(() -> getCurrentAmps() > STALL_AMPS).debounce(0.15, DebounceType.kRising);

  public ExampleRoller() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = STATOR_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    Hardware.configure("ExampleRoller", () -> motor.getConfigurator().apply(config));
  }

  // ---- Commands. Verbs, no "Command" on the end, and they all name themselves ----

  public Command intake() {
    return runVolts(INTAKE_VOLTS).until(hasPiece).withName("ExampleRoller.intake");
  }

  public Command eject() {
    return runVolts(EJECT_VOLTS).withTimeout(0.5).withName("ExampleRoller.eject");
  }

  // the default command (set in RobotContainer)
  public Command stop() {
    return runVolts(0).withName("ExampleRoller.stop");
  }

  private Command runVolts(double volts) {
    return startEnd(
        () -> motor.setControl(voltageRequest.withOutput(volts)),
        () -> motor.setControl(voltageRequest.withOutput(0)));
  }

  // ---- State ----

  @Logged
  public double getCurrentAmps() {
    return statorCurrent.refresh().getValueAsDouble();
  }

  @Logged
  public boolean hasPiece() {
    return hasPiece.getAsBoolean();
  }
}
