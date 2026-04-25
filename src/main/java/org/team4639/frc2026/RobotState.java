/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026;

import edu.wpi.first.math.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.Constants.Mode;
import org.team4639.frc2026.commands.SuperstructureCommands;
import org.team4639.frc2026.constants.led.Patterns;
import org.team4639.frc2026.constants.shooter.LookupTables;
import org.team4639.frc2026.constants.shooter.PassingTargets;
import org.team4639.frc2026.constants.shooter.ScoringState;
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
import org.team4639.frc2026.util.FMSUtil;
import org.team4639.frc2026.util.ValueCacher;
import org.team4639.lib.led.pattern.LEDPattern;
import org.team4639.lib.util.PoseEstimator;
import org.team4639.lib.util.VirtualSubsystem;
import org.team4639.lib.util.geometry.AllianceFlipUtil;
import org.team4639.lib.util.geometry.GeomUtil;

/**
 * RobotState handles all information involving the current state of the robot.
 *
 * <p>
 * Pose: all poses are reported relative to *our* alliance wall, not the blue
 * alliance wall. This is done to simplify internal calculations, however there
 * are methods available to access the true on-field pose mainly for interplay
 * with vision, but they should be used sparingly.
 */
@ExtensionMethod(GeomUtil.class)
public class RobotState extends VirtualSubsystem implements VisionConsumer, TurretCamera.TurretVisionConsumer {

    private static RobotState instance = new RobotState();

    public static final double TRUST_VISION_NORMAL = 1.0;

    public static final Trigger disabled = RobotModeTriggers.disabled();

    public static synchronized RobotState getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, RobotState::new);
    }

    double poseBufferSizeSec = 2.0;

    // SmartDashboard / Logger keys
    private final String ROBOT_FIELD_INTERNAL_KEY = "/Internal/Robot Pose";
    private final String ROBOT_FIELD_TRUE_KEY = "/RobotState/Robot Pose";
    private final String CHOREO_SETPOINT_KEY = "/Internal/Choreo Setpoint";

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    private final TimeInterpolatableBuffer<Pose2d> odometryBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    private final TimeInterpolatableBuffer<Pose2d> choreoSetpoints = TimeInterpolatableBuffer.createBuffer(0.05);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Drive.getModuleTranslations());

    private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    @Setter
    @Getter
    private boolean disableTurretCamera = false;

    @Setter
    @Getter
    private boolean disableBottomCameras = false;

    /** Assume gyro starts at zero. */
    private Rotation2d gyroOffset = Rotation2d.kZero;

    @Getter
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    @Getter
    private int turretCameraTargets = 0;

    @Setter
    @Getter
    private double gyroRotationsPerSecond;

    @Getter
    private ScoringState scoringState = new ScoringState(0, 0, 0);

    @Getter
    private double RPMFudge = 1;

    private final ValueCacher<Object, ScoringState> currentScoringStates =
            new ValueCacher<>(this::_calculateScoringState);

    private final ValueCacher<Object, ScoringState> nextScoringStates =
            new ValueCacher<>(() -> this._calculateNextScoringState(0.02));

    private final ValueCacher<Object, ScoringState> passingStates = new ValueCacher<>(this::_calculatePassingState);

    private final ValueCacher<Object, ScoringState> nextPassingStates =
            new ValueCacher<>(() -> this._calculateNextPassingState(0.02));

    private final ValueCacher<Object, Pose2d> getNextPose = new ValueCacher<>(() -> calculateNextPose(0.02));

    @Setter
    @Getter
    private Pair<Hood.WantedState, Hood.SystemState> hoodStates;

    @Setter
    @Getter
    private Pair<Shooter.WantedState, Shooter.SystemState> shooterStates;

    @Setter
    @Getter
    private Pair<Turret.WantedState, Turret.SystemState> turretStates;

    @Setter
    @Getter
    private Pair<Intake.WantedState, Intake.SystemState> intakeStates;

    @Setter
    @Getter
    private Pair<Extension.WantedState, Extension.SystemState> extensionStates;

    @Setter
    @Getter
    private Pair<Kicker.WantedState, Kicker.SystemState> kickerStates;

    @Setter
    @Getter
    private Pair<Spindexer.WantedState, Spindexer.SystemState> spindexerStates;

    private final PoseEstimator primaryPoseEstimator = new PoseEstimator(poseBufferSizeSec);
    private final PoseEstimator secondaryPoseEstimator = new PoseEstimator(poseBufferSizeSec);

    private final TimeInterpolatableBuffer<Double> turretRotations =
            TimeInterpolatableBuffer.createDoubleBuffer(poseBufferSizeSec);

    @AutoLogOutput(key = "Using Turret Buffer")
    public boolean useTurretBuffer = true;

    @Setter
    private boolean sendVisionToPrimaryPoseEstimator = true;

    @Setter
    private double visionStandardDeviationMultiplier = TRUST_VISION_NORMAL;

    @Getter
    @AutoLogOutput(key = "Intake Extension Fraction")
    private double intakeExtensionFraction = 0.0;

    @Accessors(fluent = true)
    @Getter
    private boolean useIntakeProtection = true;

    private final Queue<Boolean> canIsConnected = new LinkedList<>();
    private final Queue<Boolean> temperaturesAreFine = new LinkedList<>();

    private final Field2d robotFieldInternal = new Field2d();
    private final Field2d robotFieldTrue = new Field2d();

    @Getter
    private double turretToGoal = Double.POSITIVE_INFINITY;

    @Override
    public void periodic() {
        robotFieldInternal.setRobotPose(getEstimatedPose());
        robotFieldInternal.getObject("Turret Pose").setPose(getTurretPose());
        SmartDashboard.putData(ROBOT_FIELD_INTERNAL_KEY, robotFieldInternal);
        robotFieldTrue.setRobotPose(getTrueOnFieldPose());
        SmartDashboard.putData(ROBOT_FIELD_TRUE_KEY, robotFieldTrue);

        this.turretToGoal =
                getTurretPose().getTranslation().getDistance(FieldConstants.Hub.innerCenterPoint.toTranslation2d());

        SmartDashboard.putNumber("Turret to Goal", turretToGoal);
        SmartDashboard.putNumber(
                "Turret to Passing",
                getSecondaryEstimatedPose()
                        .getTranslation()
                        .getDistance(getSecondaryEstimatedPose()
                                .nearest(Stream.of(PassingTargets.LEFT, PassingTargets.RIGHT)
                                        .map(t -> new Pose2d(t, Rotation2d.kZero))
                                        .collect(Collectors.toSet()))
                                .getTranslation()));

        SmartDashboard.putBoolean("CAN Measurements", canIsConnected.stream().allMatch(measurement -> measurement));
        SmartDashboard.putBoolean(
                "Motor Temperatures", temperaturesAreFine.stream().allMatch(measurement -> measurement));
        SmartDashboard.putString("LED Color", getLEDColor().toHexString());
        SmartDashboard.putString(
                "Won Auto", getSmartDashboardColorFromAutoAlliance().toHexString());
        SmartDashboard.putBoolean("Turret Cam Disabled", disableTurretCamera);
        SmartDashboard.putBoolean("Bottom Cams Disabled", disableBottomCameras);

        canIsConnected.clear();
        temperaturesAreFine.clear();
    }

    @Override
    public void periodicAfterScheduler() {
        Logger.recordOutput(ROBOT_FIELD_INTERNAL_KEY, getEstimatedPose());
        Logger.recordOutput(ROBOT_FIELD_TRUE_KEY, getTrueOnFieldPose());
        Logger.recordOutput("/Internal/Secondary", getSecondaryEstimatedPose());
        if (!choreoSetpoints.getInternalBuffer().isEmpty()) {
            Logger.recordOutput(
                    CHOREO_SETPOINT_KEY,
                    choreoSetpoints.getInternalBuffer().lastEntry().getValue());
        }
    }

    public Pose2d getEstimatedPose() {
        return primaryPoseEstimator.getEstimatedPose();
    }

    public Pose2d getSecondaryEstimatedPose() {
        return secondaryPoseEstimator.getEstimatedPose();
    }

    // use secondary because we always want vision on the turret
    public Pose2d getTurretPose() {
        return secondaryPoseEstimator
                .getEstimatedPose()
                .transformBy(new Transform2d(
                        Constants.SimConstants.originToTurretRotation.toTranslation2d(),
                        Rotation2d.fromRotations(getScoringState().turretRotations())));
    }

    /**
     * Returns the pose relative to the blue alliance wall. Should be used
     * sparingly; for all internal calculations, use
     * {@link RobotState#getEstimatedPose()} instead.
     */
    public Pose2d getTrueOnFieldPose() {
        return AllianceFlipUtil.apply(getEstimatedPose());
    }

    public void resetPose(Pose2d pose) {
        primaryPoseEstimator.resetPose(pose);
        secondaryPoseEstimator.resetPose(pose);
        if (Constants.currentMode == Mode.SIM) SimRobot.getInstance().resetPose(pose);
    }

    public void addOdometryObservation(
            SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {
        primaryPoseEstimator.addOdometryMeasurement(wheelPositions, gyroAngle, timestamp);
        secondaryPoseEstimator.addOdometryMeasurement(wheelPositions, gyroAngle, timestamp);
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

    @Override
    public void accept(
            int cameraIndex,
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        if ((disableTurretCamera && cameraIndex == 999) || (disableBottomCameras && cameraIndex != 999)) {

        } else {
            secondaryPoseEstimator.addVisionObservation(
                    cameraIndex,
                    AllianceFlipUtil.apply(visionRobotPoseMeters),
                    timestampSeconds,
                    visionMeasurementStdDevs.times(visionStandardDeviationMultiplier));
            if (sendVisionToPrimaryPoseEstimator)
                primaryPoseEstimator.addVisionObservation(
                        cameraIndex,
                        AllianceFlipUtil.apply(visionRobotPoseMeters),
                        timestampSeconds,
                        visionMeasurementStdDevs.times(visionStandardDeviationMultiplier));
        }
    }

    @Override
    public void acceptTurretVision(
            int cameraIndex,
            Pose2d visionTurretPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs,
            int numtargets) {

        var estimatedRobotPose = visionTurretPoseMeters.transformBy(new Transform2d(
                        Constants.SimConstants.originToTurretRotation.toTranslation2d(),
                        useTurretBuffer
                                ? Rotation2d.fromRotations(turretRotations
                                        .getSample(timestampSeconds)
                                        .orElse(getScoringState().turretRotations()))
                                : Rotation2d.fromRotations(getScoringState().turretRotations()))
                .inverse());

        this.turretCameraTargets = numtargets;
        accept(999, estimatedRobotPose, timestampSeconds, visionMeasurementStdDevs);
    }

    public void addTurretRotationMeasurement(double timestamp, double rotations) {
        turretRotations.addSample(timestamp, rotations);
    }

    public void updateChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public void updateShooterState(double shooterRPM, double hoodDegrees, double turretRotations) {
        scoringState = scoringState.replace(shooterRPM, hoodDegrees, turretRotations);
    }

    public ScoringState calculateScoringState(Object caller) {
        return currentScoringStates.get(caller);
    }

    private ScoringState _calculateScoringState() {
        return LookupTables.getScoringState(
                getSecondaryEstimatedPose(), getChassisSpeeds(), FieldConstants.Hub.innerCenterPoint.toTranslation2d());
    }

    public ScoringState calculateNextScoringState(Object caller) {
        return nextScoringStates.get(caller);
    }

    private ScoringState _calculateNextScoringState(double dtSeconds) {
        return LookupTables.getScoringState(
                getSecondaryEstimatedPose()
                        .exp(ChassisSpeeds.fromFieldRelativeSpeeds(
                                        getChassisSpeeds(), getEstimatedPose().getRotation())
                                .toTwist2d(dtSeconds)),
                getChassisSpeeds(),
                FieldConstants.Hub.innerCenterPoint.toTranslation2d());
    }

    public ScoringState calculatePassingState(Object caller) {
        return passingStates.get(caller);
    }

    private ScoringState _calculatePassingState() {
        return LookupTables.getPassingState(
                getEstimatedPose(),
                getChassisSpeeds(),
                getSecondaryEstimatedPose()
                        .nearest(Stream.of(PassingTargets.LEFT, PassingTargets.RIGHT)
                                .map(t -> new Pose2d(t, Rotation2d.kZero))
                                .collect(Collectors.toSet()))
                        .getTranslation());
    }

    public ScoringState calculateNextPassingState(Object caller) {
        return nextPassingStates.get(caller);
    }

    private ScoringState _calculateNextPassingState(double dtSeconds) {
        return LookupTables.getPassingState(
                getSecondaryEstimatedPose()
                        .exp(ChassisSpeeds.fromFieldRelativeSpeeds(
                                        getChassisSpeeds(), getEstimatedPose().getRotation())
                                .toTwist2d(dtSeconds)),
                getChassisSpeeds(),
                getSecondaryEstimatedPose()
                        .nearest(Stream.of(PassingTargets.LEFT, PassingTargets.RIGHT)
                                .map(t -> new Pose2d(t, Rotation2d.kZero))
                                .collect(Collectors.toSet()))
                        .getTranslation());
    }

    public boolean passingWillHitHub() {
        var y = getTurretPose().getY();
        return FieldConstants.LinesHorizontal.center - FieldConstants.Hub.width / 2.0 < y
                && y < FieldConstants.LinesHorizontal.center + FieldConstants.Hub.width / 2.0;
    }

    /** Attempt to find the best hub for the turret to track while idling. */
    public Rotation2d getBestHubTrackFieldRelative() {
        var ourHub = FieldConstants.Hub.topCenterPoint.toTranslation2d();
        var opponentHub = new Translation2d(
                FieldConstants.fieldLength - ourHub.getX(), FieldConstants.fieldWidth - ourHub.getY());
        var nearestHub = getTurretPose().getTranslation().nearest(Set.of(ourHub, opponentHub));
        return nearestHub.minus(getTurretPose().getTranslation()).getAngle();
    }

    public void acceptCANMeasurement(boolean isConnected) {
        this.canIsConnected.add(isConnected);
    }

    public void acceptTemperatureMeasurement(double tempCelsius) {
        this.temperaturesAreFine.add(tempCelsius < 100);
    }

    public void updateIntakePosition(double intakeExtensionFraction) {
        this.intakeExtensionFraction = intakeExtensionFraction;
    }

    public boolean withinShooterRange() {
        var rpm = calculateScoringState("Within Shooter Range").shooterRPM();
        return LookupTables.MIN_SCORING_RPM < rpm && rpm < LookupTables.MAX_SCORING_RPM;
    }

    public LEDPattern getDesiredLEDPattern() {
        if (SuperstructureCommands.turretDisabled) return Patterns.DEFAULT;
        return switch (SuperstructureCommands.currentState) {
            case IDLE -> {
                if (intakeStates.getSecond() == Intake.SystemState.INTAKE) {
                    yield Patterns.DEFAULT_INTAKE;
                } else {
                    yield Patterns.DEFAULT;
                }
            }
            case PASS -> {
                if (intakeStates.getSecond() == Intake.SystemState.INTAKE) {
                    yield Patterns.PASSING_AND_INTAKE;
                } else {
                    yield Patterns.PASSING;
                }
            }
            case SCORE -> {
                if (intakeStates.getSecond() == Intake.SystemState.INTAKE) {
                    yield Patterns.SHOOTING_AND_INTAKE;
                } else {
                    yield Patterns.SHOOTING;
                }
            }
            case WAIT -> {
                if (intakeStates.getSecond() == Intake.SystemState.INTAKE) {
                    yield Patterns.SHOOTER_REQUESTED_AND_INTAKE;
                } else {
                    yield Patterns.SHOOTER_REQUESTED;
                }
            }
        };
    }

    private Color getLEDColor() {
        if (SuperstructureCommands.turretDisabled) return Color.kViolet;
        return switch (SuperstructureCommands.currentState) {
            case IDLE -> Color.kOrange;
            case PASS -> Color.kWhite;
            case SCORE -> Color.kGreen;
            case WAIT -> Color.kBlue;
        };
    }

    private Color getSmartDashboardColorFromAutoAlliance() {
        FMSUtil.FMSGameData data = FMSUtil.getAutoWinningAlliance();
        return switch (data.status()) {
            case OK -> data.wonAuto() == DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                    ? Color.kGreen
                    : Color.kRed;
            case NO_DS_ALLIANCE -> Color.kBlack;
            case NO_GAME_DATA -> Color.kGray;
        };
    }
}
