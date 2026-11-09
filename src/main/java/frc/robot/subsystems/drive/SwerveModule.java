package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Hardware;

// One MK5n corner. Not a subsystem, Drive owns all four and is the only thing that talks to them.
final class SwerveModule {
  private final String name;

  private final SparkMax driveMotor;
  private final RelativeEncoder driveEncoder;
  private final SparkClosedLoopController driveController;

  private final TalonFX steerMotor;
  private final CANcoder encoder;
  private final StatusSignal<Angle> steerPosition;
  private final StatusSignal<AngularVelocity> steerVelocity;
  private final StatusSignal<Angle> absolutePosition;
  private final PositionVoltage steerRequest = new PositionVoltage(0);

  private SwerveModuleState desiredState = new SwerveModuleState();

  // sim doesn't have real motors, so in sim the module just does whatever it was told
  private double simDriveMeters = 0;
  private double simDriveVelocity = 0;
  private Rotation2d simAngle = Rotation2d.kZero;

  SwerveModule(ModuleSpec spec) {
    name = spec.name();

    driveMotor = new SparkMax(spec.driveId(), MotorType.kBrushless);
    var driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .inverted(DRIVE_INVERTED)
        .smartCurrentLimit(DRIVE_CURRENT_LIMIT_AMPS)
        .voltageCompensation(12.0);
    // The NEO's built-in velocity is filtered over 32ms by default, which is way too laggy for a
    // velocity loop. 10ms / depth 2 is what most swerve teams run
    driveConfig
        .encoder
        .positionConversionFactor(DRIVE_METERS_PER_MOTOR_ROTATION)
        .velocityConversionFactor(DRIVE_METERS_PER_MOTOR_ROTATION / 60.0)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(DRIVE_KP, 0, 0);
    // Persist so a SPARK that browns out and reboots mid-match comes back with its current limit
    // and inversion, not factory defaults
    Hardware.configureRev(
        name + " drive",
        () ->
            driveMotor.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    driveEncoder.setPosition(0);

    encoder = new CANcoder(spec.encoderId());
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = spec.encoderOffsetRotations();
    encoderConfig.MagnetSensor.SensorDirection =
        ENCODER_CLOCKWISE_POSITIVE
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    Hardware.configure(name + " CANcoder", () -> encoder.getConfigurator().apply(encoderConfig));

    // The Talon closes the steering loop on the CANcoder directly (RemoteCANcoder, works without
    // Phoenix Pro), so the angle loop runs at 1kHz on the motor instead of 50Hz on the RIO
    steerMotor = new TalonFX(spec.steerId());
    var steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted =
        STEER_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    steerConfig.CurrentLimits.StatorCurrentLimit = STEER_STATOR_LIMIT_AMPS;
    steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    steerConfig.CurrentLimits.SupplyCurrentLimit = STEER_SUPPLY_LIMIT_AMPS;
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerConfig.Feedback.FeedbackRemoteSensorID = spec.encoderId();
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfig.Feedback.SensorToMechanismRatio = 1.0;
    steerConfig.Feedback.RotorToSensorRatio = STEER_GEAR_RATIO;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.Slot0.kP = STEER_KP;
    steerConfig.Slot0.kD = STEER_KD;
    Hardware.configure(name + " steer", () -> steerMotor.getConfigurator().apply(steerConfig));

    steerPosition = steerMotor.getPosition();
    steerVelocity = steerMotor.getVelocity();
    absolutePosition = encoder.getAbsolutePosition();
  }

  void refreshSignals() {
    BaseStatusSignal.refreshAll(steerPosition, steerVelocity, absolutePosition);
  }

  Rotation2d getAngle() {
    if (RobotBase.isSimulation()) {
      return simAngle;
    }
    return Rotation2d.fromRotations(
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(steerPosition, steerVelocity));
  }

  // How far the drive motor turned just because the module turned (see COUPLE_RATIO). Steer
  // position here is continuous, not wrapped, so this keeps adding up across full spins
  private double couplingMeters() {
    return steerPosition.getValueAsDouble() * COUPLE_RATIO * DRIVE_METERS_PER_MOTOR_ROTATION;
  }

  private double couplingVelocity() {
    return steerVelocity.getValueAsDouble() * COUPLE_RATIO * DRIVE_METERS_PER_MOTOR_ROTATION;
  }

  SwerveModulePosition getPosition() {
    if (RobotBase.isSimulation()) {
      return new SwerveModulePosition(simDriveMeters, simAngle);
    }
    return new SwerveModulePosition(driveEncoder.getPosition() - couplingMeters(), getAngle());
  }

  SwerveModuleState getState() {
    if (RobotBase.isSimulation()) {
      return new SwerveModuleState(simDriveVelocity, simAngle);
    }
    return new SwerveModuleState(driveEncoder.getVelocity() - couplingVelocity(), getAngle());
  }

  SwerveModuleState getDesiredState() {
    return desiredState;
  }

  void setDesiredState(SwerveModuleState state) {
    Rotation2d angle = getAngle();
    // never turn more than 90 degrees, just drive backwards instead. Then slow the wheel down while
    // it's still pointed the wrong way so the robot doesn't lurch sideways
    state.optimize(angle);
    state.cosineScale(angle);
    desiredState = state;

    double wheelSpeed = state.speedMetersPerSecond;
    double motorSpeed = wheelSpeed + couplingVelocity();
    double feedforward = DRIVE_KS * Math.signum(wheelSpeed) + DRIVE_KV * motorSpeed;
    driveController.setSetpoint(
        motorSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kVoltage);
    steerMotor.setControl(steerRequest.withPosition(state.angle.getRotations()));
  }

  // Points the module somewhere with the drive motor at 0V. Only used by the coupling test
  void setAngleOnly(Rotation2d angle) {
    desiredState = new SwerveModuleState(0, angle);
    driveMotor.setVoltage(0);
    steerMotor.setControl(steerRequest.withPosition(angle.getRotations()));
  }

  void stop() {
    driveMotor.stopMotor();
    steerMotor.stopMotor();
  }

  void simulationPeriodic(double dtSeconds) {
    simAngle = desiredState.angle;
    simDriveVelocity = desiredState.speedMetersPerSecond;
    simDriveMeters += simDriveVelocity * dtSeconds;
  }

  // ---- bring-up numbers, logged by Drive ----

  double getAbsoluteRotations() {
    return absolutePosition.getValueAsDouble();
  }

  double getRawDriveMeters() {
    return driveEncoder.getPosition();
  }

  double getDriveCurrentAmps() {
    return driveMotor.getOutputCurrent();
  }
}
