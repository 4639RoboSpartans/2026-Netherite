/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.*;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.Constants.Mode;
import org.team4639.frc2026.constants.led.Patterns;
import org.team4639.frc2026.constants.shooter.PassingLookupTable;
import org.team4639.frc2026.constants.shooter.ScoringState;
import org.team4639.frc2026.constants.shooter.ShooterScoringData;
import org.team4639.frc2026.subsystems.drive.Drive;
import org.team4639.frc2026.subsystems.extension.Extension;
import org.team4639.frc2026.subsystems.hood.Hood;
import org.team4639.frc2026.subsystems.intake.Intake;
import org.team4639.frc2026.subsystems.kicker.Kicker;
import org.team4639.frc2026.subsystems.shooter.Shooter;
import org.team4639.frc2026.subsystems.spindexer.Spindexer;
import org.team4639.frc2026.subsystems.turret.Turret;
import org.team4639.frc2026.subsystems.vision.TurretCamera;
import org.team4639.frc2026.subsystems.vision.Vision.VisionConsumer;
import org.team4639.frc2026.util.ValueCacher;
import org.team4639.lib.led.pattern.LEDPattern;
import org.team4639.lib.unit.Units2;
import org.team4639.lib.util.VirtualSubsystem;
import org.team4639.lib.util.geometry.AllianceFlipUtil;
import org.team4639.lib.util.geometry.GeomUtil;

/**
 * RobotState handles all information involving the current state of the robot.
 *
 * <p>Pose: all poses are reported relative to *our* alliance wall, not the blue alliance wall. This
 * is done to simplify internal calculations, however there are methods available to access the true
 * on-field pose mainly for interplay with vision, but they should be used sparingly.
 */
@ExtensionMethod(GeomUtil.class)
public class RobotState extends VirtualSubsystem
        implements VisionConsumer, TurretCamera.TurretVisionConsumer {

  // -------------------------------------------------------------------------
  // Singleton
  // -------------------------------------------------------------------------

  private static RobotState instance = new RobotState();

  public static synchronized RobotState getInstance() {
    return instance = Objects.requireNonNullElseGet(instance, RobotState::new);
  }

  // -------------------------------------------------------------------------
  // Records
  // -------------------------------------------------------------------------

  private record OdometryObservation(
          SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

  private record VisionObservation(
          int camIndex, Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

  // -------------------------------------------------------------------------
  // Constants & Configuration
  // -------------------------------------------------------------------------

  double poseBufferSizeSec = 2.0;

  private final Matrix<N3, N1> qStdDevs =
          new Matrix<>(VecBuilder.fill(0.000009, 0.000009, 0.000004));

  // SmartDashboard / Logger keys
  private final String ROBOT_FIELD_INTERNAL_KEY = "/Internal/Robot Pose";
  private final String ROBOT_FIELD_TRUE_KEY = "/RobotState/Robot Pose";
  private final String CHOREO_SETPOINT_KEY = "/Internal/Choreo Setpoint";

  // -------------------------------------------------------------------------
  // Pose Buffers
  // -------------------------------------------------------------------------

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
          TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

  private final TimeInterpolatableBuffer<Pose2d> odometryBuffer =
          TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

  private final Map<Integer, TimeInterpolatableBuffer<Pose2d>> visionPoseBuffers = new HashMap<>();

  private final TimeInterpolatableBuffer<Pose2d> choreoSetpoints =
          TimeInterpolatableBuffer.createBuffer(0.05);

  // -------------------------------------------------------------------------
  // Odometry State
  // -------------------------------------------------------------------------

  private final SwerveDriveKinematics kinematics =
          new SwerveDriveKinematics(Drive.getModuleTranslations());

  private SwerveModulePosition[] lastWheelPositions =
          new SwerveModulePosition[] {
                  new SwerveModulePosition(),
                  new SwerveModulePosition(),
                  new SwerveModulePosition(),
                  new SwerveModulePosition()
          };

  /** Assume gyro starts at zero. */
  private Rotation2d gyroOffset = Rotation2d.kZero;

  @AutoLogOutput @Getter private Pose2d odometryPose = Pose2d.kZero;

  /** Pose <b>relative to our alliance wall</b>. */
  @AutoLogOutput @Getter private Pose2d estimatedPose = Pose2d.kZero;

  // -------------------------------------------------------------------------
  // Chassis Speeds
  // -------------------------------------------------------------------------

  @Getter private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  // -------------------------------------------------------------------------
  // Turret State
  // -------------------------------------------------------------------------

  @Getter
  @AutoLogOutput(key = "Turret Pose")
  private Pose2d turretPose = new Pose2d();

  @Getter private int turretCameraTargets = 0;

  @Setter @Getter private double gyroRotationsPerSecond;

  private final TimeInterpolatableBuffer<Double> turretRobotRelativeBuffer =
          TimeInterpolatableBuffer.createDoubleBuffer(poseBufferSizeSec);

  // -------------------------------------------------------------------------
  // Scoring & Shooting State
  // -------------------------------------------------------------------------

  @Getter
  private ScoringState scoringState =
          new ScoringState(Units2.RPM.of(0.0), Rotations.of(0), Rotations.of(0));

  @Getter private double RPMFudge = 1;

  private final ValueCacher<Object, ScoringState> currentScoringStates =
          new ValueCacher<>(this::_calculateScoringState);

  private final ValueCacher<Object, ScoringState> nextScoringStates =
          new ValueCacher<>(() -> this._calculateNextScoringState(0.02));

  private final ValueCacher<Object, ScoringState> passingStates =
          new ValueCacher<>(this::_calculatePassingState);

  private final ValueCacher<Object, Pose2d> getNextPose =
          new ValueCacher<>(() -> calculateNextPose(0.02));

  // -------------------------------------------------------------------------
  // Subsystem State Pairs
  // -------------------------------------------------------------------------

  @Setter @Getter private Pair<Hood.WantedState, Hood.SystemState> hoodStates;
  @Setter @Getter private Pair<Shooter.WantedState, Shooter.SystemState> shooterStates;
  @Setter @Getter private Pair<Turret.WantedState, Turret.SystemState> turretStates;
  @Setter @Getter private Pair<Intake.WantedState, Intake.SystemState> intakeStates;
  @Setter @Getter private Pair<Extension.WantedState, Extension.SystemState> extensionStates;
  @Setter @Getter private Pair<Kicker.WantedState, Kicker.SystemState> kickerStates;
  @Setter @Getter private Pair<Spindexer.WantedState, Spindexer.SystemState> spindexerStates;

  // -------------------------------------------------------------------------
  // Miscellaneous Robot State
  // -------------------------------------------------------------------------

  @Setter private GenericSuperstructureState superstructureState = GenericSuperstructureState.IDLE;

  @Getter
  @AutoLogOutput(key = "Intake Extension Fraction")
  private double intakeExtensionFraction = 0.0;

  @Accessors(fluent = true)
  @Getter
  private boolean useIntakeProtection = true;

  private final Queue<Boolean> canIsConnected = new LinkedList<>();
  private final Queue<Boolean> temperaturesAreFine = new LinkedList<>();

  // -------------------------------------------------------------------------
  // SmartDashboard / Field Display Objects
  // -------------------------------------------------------------------------

  private final Field2d robotFieldInternal = new Field2d();
  private final Field2d robotFieldTrue = new Field2d();

  // =========================================================================
  // Lifecycle Methods
  // =========================================================================

  @Override
  public void periodic() {
    robotFieldInternal.setRobotPose(estimatedPose);
    robotFieldInternal.getObject("Turret Pose").setPose(turretPose);
    SmartDashboard.putData(ROBOT_FIELD_INTERNAL_KEY, robotFieldInternal);
    robotFieldTrue.setRobotPose(getTrueOnFieldPose());
    SmartDashboard.putData(ROBOT_FIELD_TRUE_KEY, robotFieldTrue);

    ScoringState scoringState = calculateScoringState(this);
    SmartDashboard.putNumber("Scoring/CalculatedRPM", scoringState.shooterRPM().in(Units2.RPM));
    SmartDashboard.putNumber("Scoring/CalculatedHoodAngle", scoringState.hoodAngle().in(Rotations));
    SmartDashboard.putNumber(
            "Scoring/CalculatedTurretAngle", scoringState.turretAngle().in(Rotations));
    SmartDashboard.putNumber(
            "Scoring/CalculatedTurretAngleFieldRelative",
            getEstimatedPose().getRotation().getMeasure().plus(scoringState.turretAngle()).in(Degrees));

    SmartDashboard.putNumber(
            "Turret to Goal",
            getTurretPose()
                    .getTranslation()
                    .getDistance(FieldConstants.Hub.innerCenterPoint.toTranslation2d()));
    SmartDashboard.putNumber(
            "Turret to Passing Line",
            getTurretPose().getX() - FieldConstants.LinesVertical.allianceZone / 2);

    ShooterScoringData.shooterLookupTable.calculateShooterStateStationary(
            getTurretPose(), FieldConstants.Hub.innerCenterPoint.toTranslation2d());

    SmartDashboard.putBoolean(
            "CAN Measurements", canIsConnected.stream().allMatch(measurement -> measurement));
    SmartDashboard.putBoolean(
            "Motor Temperatures", temperaturesAreFine.stream().allMatch(measurement -> measurement));

    canIsConnected.clear();
    temperaturesAreFine.clear();
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput(ROBOT_FIELD_INTERNAL_KEY, estimatedPose);
    Logger.recordOutput(ROBOT_FIELD_TRUE_KEY, getTrueOnFieldPose());
    if (!choreoSetpoints.getInternalBuffer().isEmpty()) {
      Logger.recordOutput(
              CHOREO_SETPOINT_KEY, choreoSetpoints.getInternalBuffer().lastEntry().getValue());
    }
  }

  // =========================================================================
  // Pose / Odometry Methods
  // =========================================================================

  /**
   * Returns the pose relative to the blue alliance wall. Should be used sparingly; for all
   * internal calculations, use {@link RobotState#getEstimatedPose()} instead.
   */
  public Pose2d getTrueOnFieldPose() {
    return AllianceFlipUtil.apply(getEstimatedPose());
  }

  public void resetPose(Pose2d pose) {
    poseBuffer.clear();
    estimatedPose = pose;
    if (Constants.currentMode == Mode.SIM) SimRobot.getInstance().resetPose(pose);
  }

  public void resetGyro() {
    resetPose(getEstimatedPose().withRotation(new Rotation2d()));
  }

  public void addOdometryObservation(
          SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {
    addOdometryMeasurement(new OdometryObservation(wheelPositions, gyroAngle, timestamp));
  }

  private void addOdometryMeasurement(OdometryObservation observation) {
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();

    if (odometryBuffer.getInternalBuffer().isEmpty())
      odometryBuffer.addSample(observation.timestamp(), Pose2d.kZero);
    else
      odometryBuffer.addSample(
              observation.timestamp,
              odometryBuffer
                      .getInternalBuffer()
                      .get(odometryBuffer.getInternalBuffer().lastKey())
                      .exp(twist));

    Pose2d lastOdometryPose = odometryPose;
    odometryPose = odometryPose.exp(twist);
    observation.gyroAngle.ifPresent(
            gyroAngle -> {
              Rotation2d angle = gyroAngle.plus(gyroOffset);
              odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
            });
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);

    turretPose =
            estimatedPose.transformBy(
                    new Transform2d(
                            Constants.SimConstants.originToTurretRotation.toTranslation2d(),
                            Rotation2d.fromRotations(getScoringState().turretAngle().in(Rotations))));
  }

  public Pose2d calculateNextPose(Object caller) {
    return getNextPose.get(caller);
  }

  private Pose2d calculateNextPose(double dt) {
    return getEstimatedPose().exp(chassisSpeeds.toTwist2d(dt));
  }

  public void setChoreoSetpoint(Pose2d pose) {
    choreoSetpoints.addSample(Timer.getTimestamp(), pose);
  }

  public Optional<Pose2d> getChoreoSetpoint() {
    if (choreoSetpoints.getInternalBuffer().isEmpty()) {
      return Optional.empty();
    } else {
      return Optional.of(choreoSetpoints.getInternalBuffer().lastEntry().getValue());
    }
  }

  public Rotation2d[] calculateClosestDriveAndTurretRotation(ScoringState desiredScoringState) {
    Rotation2d currentRobotRotationFieldRelative = getEstimatedPose().getRotation();
    Rotation2d turretDirectionFieldRelative = new Rotation2d(scoringState.turretAngle());
    double minDriveDirectionRadians = turretDirectionFieldRelative.getRadians();
    double maxDriveDirectionRadians = turretDirectionFieldRelative.getRadians() + Math.PI * 3 / 4;
    if (minDriveDirectionRadians < currentRobotRotationFieldRelative.getRadians()
            && currentRobotRotationFieldRelative.getRadians() < maxDriveDirectionRadians) {
      return new Rotation2d[] {
              currentRobotRotationFieldRelative,
              turretDirectionFieldRelative.minus(currentRobotRotationFieldRelative)
      };
    } else if (minDriveDirectionRadians > currentRobotRotationFieldRelative.getRadians()) {
      return new Rotation2d[] {
              Rotation2d.fromRadians(minDriveDirectionRadians),
              turretDirectionFieldRelative.minus(Rotation2d.fromRadians(minDriveDirectionRadians))
      };
    } else {
      return new Rotation2d[] {
              Rotation2d.fromRadians(maxDriveDirectionRadians),
              turretDirectionFieldRelative.minus(Rotation2d.fromRadians(maxDriveDirectionRadians))
      };
    }
  }

  public Pose3d[] getComponentPoses() {
    Pose3d turretPose = new Pose3d();
    turretPose =
            turretPose.rotateAround(
                    Constants.SimConstants.originToTurretRotation,
                    new Rotation3d(new Rotation2d(scoringState.turretAngle())));
    Pose3d hoodPose = new Pose3d();
    Rotation2d hoodRotation =
            Rotation2d.fromDegrees(
                    -scoringState.hoodAngle().in(Degrees)
                            + org.team4639.frc2026.subsystems.hood.Constants.HOOD_MIN_ANGLE_DEGREES);
    hoodPose =
            hoodPose.rotateAround(
                    Constants.SimConstants.originToHoodRotation,
                    new Rotation3d(VecBuilder.fill(0, 1, 0), -hoodRotation.getRadians()));
    hoodPose =
            hoodPose.rotateAround(
                    Constants.SimConstants.originToTurretRotation,
                    new Rotation3d(new Rotation2d(scoringState.turretAngle())));
    Pose3d intakePose =
            new Pose3d(
                    new Translation3d(Units.inchesToMeters(10.396), 0, Units.inchesToMeters(-3.277)),
                    new Rotation3d());
    return new Pose3d[] {intakePose, turretPose, hoodPose};
  }

  // =========================================================================
  // Vision Methods
  // =========================================================================

  @Override
  public void accept(
          int cameraIndex,
          Pose2d visionRobotPoseMeters,
          double timestampSeconds,
          Matrix<N3, N1> visionMeasurementStdDevs) {
    addVisionObservation(
            new VisionObservation(
                    cameraIndex,
                    AllianceFlipUtil.apply(visionRobotPoseMeters),
                    timestampSeconds,
                    visionMeasurementStdDevs));
  }

  @Override
  public void acceptTurretVision(
          int cameraIndex,
          Pose2d visionTurretPoseMeters,
          double timestampSeconds,
          Matrix<N3, N1> visionMeasurementStdDevs,
          int numtargets) {
    if (superstructureState == GenericSuperstructureState.MANUAL) return;

    var estimatedRobotPose =
            visionTurretPoseMeters.transformBy(
                    new Transform2d(
                            Constants.SimConstants.originToTurretRotation.toTranslation2d(),
                            Rotation2d.fromRotations(getScoringState().turretAngle().in(Rotations)))
                            .inverse());

    this.turretCameraTargets = numtargets;
    accept(999, estimatedRobotPose, timestampSeconds, visionMeasurementStdDevs);
  }

  private void addVisionObservation(VisionObservation observation) {
    Logger.recordOutput("Vision Obs Pose", observation.visionPose);
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) return;

    if (!visionPoseBuffers.containsKey(observation.camIndex()))
      visionPoseBuffers.put(
              observation.camIndex(), TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec));
    visionPoseBuffers
            .get(observation.camIndex())
            .addSample(observation.timestamp(), observation.visionPose());

    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }
    // Solve for closed-form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }

    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
    var kTimesTransform =
            visionK.times(
                    VecBuilder.fill(
                            transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
            new Transform2d(
                    kTimesTransform.get(0, 0),
                    kTimesTransform.get(1, 0),
                    Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    turretPose =
            estimatedPose.transformBy(
                    new Transform2d(
                            Constants.SimConstants.originToTurretRotation.toTranslation2d(),
                            Rotation2d.fromRotations(getScoringState().turretAngle().in(Rotations))));
  }

  // =========================================================================
  // Chassis Speed Methods
  // =========================================================================

  public void updateChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    lastChassisSpeeds = this.chassisSpeeds;
    this.chassisSpeeds = chassisSpeeds;
  }

  public double getAccelerationMPSSquared() {
    var x = (chassisSpeeds.vxMetersPerSecond - lastChassisSpeeds.vyMetersPerSecond) / 0.02;
    var y = (chassisSpeeds.vyMetersPerSecond - lastChassisSpeeds.vyMetersPerSecond) / 0.02;
    return Math.hypot(x, y);
  }

  // =========================================================================
  // Scoring & Shooting Methods
  // =========================================================================

  public void updateShooterState(AngularVelocity shooterRPM, Angle hoodAngle, Angle turretAngle) {
    AngularVelocity newShooterRPM = scoringState.shooterRPM();
    if (shooterRPM != null) newShooterRPM = shooterRPM;
    SmartDashboard.putNumber("Scoring/ShooterRPM", newShooterRPM.in(Units2.RPM));
    Angle newHoodAngle = scoringState.hoodAngle();
    if (hoodAngle != null) newHoodAngle = hoodAngle;
    SmartDashboard.putNumber("Scoring/HoodAngle", newHoodAngle.in(Rotations));
    Angle newTurretAngle = scoringState.turretAngle();
    if (turretAngle != null) newTurretAngle = turretAngle;
    SmartDashboard.putNumber("Scoring/TurretAngle", newTurretAngle.in(Rotations));
    scoringState = new ScoringState(newShooterRPM, newHoodAngle, newTurretAngle);
  }

  public ScoringState calculateScoringState(Object caller) {
    return currentScoringStates.get(caller);
  }

  private ScoringState _calculateScoringState() {
    Translation2d hubTranslation = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    Pose2d turretPose =
            getEstimatedPose()
                    .transformBy(
                            new Transform2d(
                                    Constants.SimConstants.originToTurretRotation.toTranslation2d(),
                                    new Rotation2d()));
    return calculateScoringState(hubTranslation, turretPose, getChassisSpeeds())
            .times(getRPMFudge(), 1, 1);
  }

  public ScoringState calculateNextScoringState(Object caller) {
    return nextScoringStates.get(caller);
  }

  private ScoringState _calculateNextScoringState(double dtSeconds) {
    Translation2d hubTranslation = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    Pose2d turretPose =
            getEstimatedPose()
                    .exp(getChassisSpeeds().toTwist2d(dtSeconds))
                    .transformBy(
                            new Transform2d(
                                    Constants.SimConstants.originToTurretRotation.toTranslation2d(),
                                    new Rotation2d()));
    return calculateScoringState(hubTranslation, turretPose, getChassisSpeeds());
  }

  private ScoringState calculateScoringState(
          Translation2d hubTranslation, Pose2d turretPose, ChassisSpeeds chassisSpeeds) {
    if (MathUtil.isNear(0, chassisSpeeds.vxMetersPerSecond, 0.01)
            || MathUtil.isNear(0, chassisSpeeds.vyMetersPerSecond, 0.01)) {
      return ShooterScoringData.shooterLookupTable.calculateShooterStateStationary(
              turretPose, hubTranslation);
    } else {
      Rotation2d robotRotation = turretPose.getRotation();
      Translation2d robotVelocity =
              new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
      Translation2d robotTangentialVelocityTranslation =
              Constants.SimConstants.originToTurretRotation
                      .toTranslation2d()
                      .rotateBy(robotRotation)
                      .rotateBy(Rotation2d.fromDegrees(90))
                      .times(chassisSpeeds.omegaRadiansPerSecond);
      robotTangentialVelocityTranslation.rotateBy(robotRotation);
      robotVelocity = robotVelocity.plus(robotTangentialVelocityTranslation);
      return ShooterScoringData.shooterLookupTable.convergeShooterStateSOTFTurret(
              turretPose, hubTranslation, robotVelocity, 10);
    }
  }

  public ScoringState calculatePassingState(Object caller) {
    return passingStates.get(caller);
  }

  private ScoringState _calculatePassingState() {
    return PassingLookupTable.getPassingState(
            getTurretPose(), FieldConstants.LinesVertical.allianceZone / 2.0);
  }

  public boolean passingWillHitHub() {
    var y = getTurretPose().getY();
    return FieldConstants.LinesHorizontal.center - FieldConstants.Hub.width / 2.0 < y
            && y < FieldConstants.LinesHorizontal.center + FieldConstants.Hub.width / 2.0;
  }

  public void fudgeUp() {
    this.RPMFudge *= 1.01;
  }

  public void fudgeDown() {
    this.RPMFudge /= 1.01;
  }

  // =========================================================================
  // Turret Methods
  // =========================================================================

  public void acceptTurretMeasurement(double rotations, double timestamp) {
    turretRobotRelativeBuffer.addSample(timestamp, rotations);
  }

  /**
   * Attempt to find the best hub for the turret to track while idling.
   */
  public Rotation2d getBestHubTrackFieldRelative() {
    var ourHub = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    var opponentHub =
            new Translation2d(
                    FieldConstants.fieldLength - ourHub.getX(), FieldConstants.fieldWidth - ourHub.getY());
    var nearestHub = getTurretPose().getTranslation().nearest(Set.of(ourHub, opponentHub));
    return nearestHub.minus(getTurretPose().getTranslation()).getAngle();
  }


  // =========================================================================
  // Hardware Health Methods
  // =========================================================================

  public void acceptCANMeasurement(boolean isConnected) {
    this.canIsConnected.add(isConnected);
  }

  public void acceptTemperatureMeasurement(double tempCelsius) {
    this.temperaturesAreFine.add(tempCelsius < 100);
  }

  // =========================================================================
  // Intake Methods
  // =========================================================================

  public void updateIntakePosition(double intakeExtensionFraction) {
    this.intakeExtensionFraction = intakeExtensionFraction;
  }

  public void toggleIntakeProtection() {
    this.useIntakeProtection = !this.useIntakeProtection;
  }

  // =========================================================================
  // LED Methods
  // =========================================================================

  public LEDPattern getDesiredLEDPattern() {
    return LEDPattern.BLANK;
  }
}