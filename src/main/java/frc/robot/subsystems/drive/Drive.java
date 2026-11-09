package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AllianceUtil;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged(strategy = Strategy.OPT_IN)
public class Drive extends SubsystemBase {
  private final SwerveModule[] modules =
      Arrays.stream(MODULES).map(SwerveModule::new).toArray(SwerveModule[]::new);
  private final Gyro gyro = Gyro.create(GYRO);
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field = new Field2d();
  private final Alert gyroDisconnected = new Alert("Gyro disconnected", AlertType.kError);

  // sim has no gyro, so we add up the commanded spin instead
  private Rotation2d simYaw = Rotation2d.kZero;

  public Drive() {
    poseEstimator =
        new SwerveDrivePoseEstimator(KINEMATICS, getGyroYaw(), getModulePositions(), Pose2d.kZero);
    SmartDashboard.putData("Field", field);
    SmartDashboard.putData("Swerve", this::initSwerveWidget);
    configurePathPlanner();
  }

  @Override
  public void periodic() {
    for (SwerveModule module : modules) {
      module.refreshSignals();
    }
    poseEstimator.update(getGyroYaw(), getModulePositions());
    field.setRobotPose(getPose());
    gyroDisconnected.set(RobotBase.isReal() && !gyro.isConnected());
  }

  @Override
  public void simulationPeriodic() {
    for (SwerveModule module : modules) {
      module.simulationPeriodic(0.02);
    }
    double omega = KINEMATICS.toChassisSpeeds(getDesiredStates()).omegaRadiansPerSecond;
    simYaw = simYaw.plus(Rotation2d.fromRadians(omega * 0.02));
  }

  // ---- Commands ----

  /**
   * Drives off the joysticks. translation is fractions of top speed (so a stick reading straight
   * in), rotation is -1 to 1 with CCW positive. Field relative is "away from the driver station
   * pushes away from you" on both alliances.
   */
  public Command joystickDrive(
      Supplier<Translation2d> translation, DoubleSupplier rotation, boolean fieldRelative) {
    return run(() -> {
          Translation2d velocity = translation.get().times(MAX_SPEED_MPS);
          var speeds =
              new ChassisSpeeds(
                  velocity.getX(),
                  velocity.getY(),
                  rotation.getAsDouble() * MAX_ANGULAR_SPEED_RAD_PER_SEC);
          if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, driverHeading());
          }
          runVelocity(speeds);
        })
        .withName(fieldRelative ? "Drive.joystickFieldRelative" : "Drive.joystickRobotRelative");
  }

  /** Drives at a fixed robot relative speed until interrupted. For quick no-path autos. */
  public Command driveRobotRelative(ChassisSpeeds speeds) {
    return run(() -> runVelocity(speeds)).withName("Drive.driveRobotRelative");
  }

  /** Makes whatever direction the robot is facing right now "away from the driver". */
  public Command zeroHeading() {
    return runOnce(
            () ->
                resetPose(
                    new Pose2d(
                        getPose().getTranslation(),
                        AllianceUtil.isRed() ? Rotation2d.k180deg : Rotation2d.kZero)))
        .ignoringDisable(true)
        .withName("Drive.zeroHeading");
  }

  /** Wheels in an X so we're hard to push. Hold it. */
  public Command xLock() {
    return run(() -> {
          for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(new SwerveModuleState(0, MODULES[i].location().getAngle()));
          }
        })
        .withName("Drive.xLock");
  }

  public Command stop() {
    return runOnce(this::stopModules).withName("Drive.stop");
  }

  /**
   * Bring-up only. Spins every module slowly in place with the drive motors at 0V, so we can check
   * the sign of COUPLE_RATIO (see README). Robot on the ground, not on blocks.
   */
  public Command couplingTest() {
    Timer timer = new Timer();
    return runOnce(timer::restart)
        .andThen(
            run(
                () -> {
                  Rotation2d angle = Rotation2d.fromRotations(timer.get() * 0.25);
                  for (SwerveModule module : modules) {
                    module.setAngleOnly(angle);
                  }
                }))
        .finallyDo(this::stopModules)
        .withName("Drive.couplingTest");
  }

  // ---- State ----

  @Logged
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // Public because PathPlanner and vision need it. Everything else should use a command
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  @Logged
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return KINEMATICS.toChassisSpeeds(getMeasuredStates());
  }

  @Logged
  public SwerveModuleState[] getMeasuredStates() {
    return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
  }

  @Logged
  public SwerveModuleState[] getDesiredStates() {
    return Arrays.stream(modules)
        .map(SwerveModule::getDesiredState)
        .toArray(SwerveModuleState[]::new);
  }

  @Logged
  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(modules)
        .map(SwerveModule::getPosition)
        .toArray(SwerveModulePosition[]::new);
  }

  @Logged
  public Rotation2d getGyroYaw() {
    return RobotBase.isSimulation() ? simYaw : gyro.getYaw();
  }

  // Raw CANcoder readings, FL FR BL BR. Read these off the dashboard when setting encoder offsets
  @Logged
  public double[] getEncoderAbsoluteRotations() {
    return Arrays.stream(modules).mapToDouble(SwerveModule::getAbsoluteRotations).toArray();
  }

  // Drive encoders before coupling gets taken out, for the coupling test
  @Logged
  public double[] getRawDriveMeters() {
    return Arrays.stream(modules).mapToDouble(SwerveModule::getRawDriveMeters).toArray();
  }

  @Logged
  public double[] getDriveCurrentsAmps() {
    return Arrays.stream(modules).mapToDouble(SwerveModule::getDriveCurrentAmps).toArray();
  }

  // ---- Internals ----

  // Elastic's SwerveDrive widget wants a Sendable shaped exactly like this: radians, FL FR BL BR
  private void initSwerveWidget(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");
    String[] names = {"Front Left", "Front Right", "Back Left", "Back Right"};
    for (int i = 0; i < modules.length; i++) {
      SwerveModule module = modules[i];
      builder.addDoubleProperty(
          names[i] + " Angle", () -> module.getState().angle.getRadians(), null);
      builder.addDoubleProperty(
          names[i] + " Velocity", () -> module.getState().speedMetersPerSecond, null);
    }
    builder.addDoubleProperty("Robot Angle", () -> getPose().getRotation().getRadians(), null);
  }

  // Which way is "away from the driver" in the robot's frame. The field origin is always the blue
  // corner, so on red that's flipped 180
  private Rotation2d driverHeading() {
    Rotation2d heading = getPose().getRotation();
    return AllianceUtil.isRed() ? heading.plus(Rotation2d.k180deg) : heading;
  }

  private void runVelocity(ChassisSpeeds robotRelative) {
    // discretize so the robot doesn't drift sideways while driving and spinning at the same time
    ChassisSpeeds speeds = ChassisSpeeds.discretize(robotRelative, 0.02);
    SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED_MPS);

    // when the sticks let go, leave the wheels pointed where they were instead of snapping them
    // all back to 0 degrees
    boolean stopped =
        Math.abs(speeds.vxMetersPerSecond) < 0.01
            && Math.abs(speeds.vyMetersPerSecond) < 0.01
            && Math.abs(speeds.omegaRadiansPerSecond) < 0.01;
    for (int i = 0; i < modules.length; i++) {
      if (stopped) {
        states[i] = new SwerveModuleState(0, modules[i].getAngle());
      }
      modules[i].setDesiredState(states[i]);
    }
  }

  private void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  private void configurePathPlanner() {
    var moduleConfig =
        new com.pathplanner.lib.config.ModuleConfig(
            WHEEL_RADIUS_METERS,
            MAX_SPEED_MPS,
            WHEEL_COF,
            DRIVE_MOTOR.withReduction(DRIVE_GEAR_RATIO),
            DRIVE_CURRENT_LIMIT_AMPS,
            1);
    var robotConfig =
        new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI_KG_M2,
            moduleConfig,
            FRONT_LEFT.location(),
            FRONT_RIGHT.location(),
            BACK_LEFT.location(),
            BACK_RIGHT.location());
    // PathPlanner paths are drawn blue side, and it mirrors them itself when we're red
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> runVelocity(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(PATH_TRANSLATION_KP, 0, 0), new PIDConstants(PATH_ROTATION_KP, 0, 0)),
        robotConfig,
        AllianceUtil::isRed,
        this);
  }
}
