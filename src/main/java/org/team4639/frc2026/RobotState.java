/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.Constants.Mode;
import org.team4639.frc2026.subsystems.drive.Drive;
import org.team4639.frc2026.subsystems.vision.Vision.VisionConsumer;
import org.team4639.lib.util.VirtualSubsystem;
import org.team4639.lib.util.geometry.AllianceFlipUtil;

import java.util.*;

/**
 * RobotState handles all information involving the current state of the robot.
 * <p>
 * Pose: all poses are reported relative to *our* alliance wall, not the
 * blue alliance wall. This is done to simplify internal calculations,
 * however there are methods available to access the true on-field pose
 * mainly for interplay with vision, but they should be used sparingly.
 */
public class RobotState extends VirtualSubsystem implements VisionConsumer {
    // Singleton
    private static RobotState instance = new RobotState();

    public static synchronized RobotState getInstance() {
        // If for some reason instance does not exist, creates new
        return instance = Objects.requireNonNullElseGet(instance, RobotState::new);
    }

    double poseBufferSizeSec = 2.0;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    private final TimeInterpolatableBuffer<Pose2d> odometryBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    private final Map<Integer, TimeInterpolatableBuffer<Pose2d>> visionPoseBuffers = new HashMap<>();

    private final Matrix<N3, N1> qStdDevs = new Matrix<>(VecBuilder.fill(0.000009, 0.000009, 0.000004));

    // Odometry
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Drive.getModuleTranslations());
    private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };
    // Assume gyro starts at zero
    private Rotation2d gyroOffset = Rotation2d.kZero;

    @AutoLogOutput
    @Getter
    private Pose2d odometryPose = Pose2d.kZero;

    @AutoLogOutput
    @Getter
    /**
     * Pose <b>relative to our alliance wall</b>
     */
    private Pose2d estimatedPose = Pose2d.kZero;
    private double intakeExtensionFraction = 0.0;

    // RobotState Field and Pose Publishers
    private final String ROBOT_FIELD_INTERNAL_KEY = "/Internal/Robot Pose";
    private final Field2d robotFieldInternal = new Field2d();
    // <i>might</i> flip internal robot pose depending on alliance
    private final String ROBOT_FIELD_TRUE_KEY = "/RobotState/Robot Pose";
    private final Field2d robotFieldTrue = new Field2d();
    private final String CHOREO_SETPOINT_KEY = "/Internal/Choreo Setpoint";

    private final TimeInterpolatableBuffer<Pose2d> choreoSetpoints = TimeInterpolatableBuffer.createBuffer(0.05);

    /**
     * Returns the pose relative to the blue alliance wall.
     * Should be used sparingly, for all internal calculations,
     * use {@link RobotState#getEstimatedPose()} instead.
     * @return
     */
    public Pose2d getTrueOnFieldPose() {
        // Flips the pose if we are on Red alliance
        // To get the true robot pose
        return AllianceFlipUtil.apply(getEstimatedPose());
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
        // Use gyro if connected
        observation.gyroAngle.ifPresent(gyroAngle -> {
            // Add offset to measured angle
            Rotation2d angle = gyroAngle.plus(gyroOffset);
            odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        });
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPose);
        // Calculate diff from last odometry pose and add onto pose estimate
        Twist2d finalTwist = lastOdometryPose.log(odometryPose);
        estimatedPose = estimatedPose.exp(finalTwist);
    }

    private void addVisionObservation(VisionObservation observation) {
        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > observation.timestamp()) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }
        // Get odometry based pose at timestamp
        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) {
            // exit if not there
            return;
        }

        // Add pose to relevant camera's buffer
        if (!visionPoseBuffers.containsKey(observation.camIndex()))
            visionPoseBuffers.put(observation.camIndex(), TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec));
        visionPoseBuffers.get(observation.camIndex()).addSample(observation.timestamp(), observation.visionPose());

        // sample --> odometryPose transform and backwards of that
        var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        // get old estimate by applying odometryToSample Transform
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Calculate 3 x 3 vision matrix
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
        }
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
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
        // difference between estimate and vision pose
        Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());

        // scale transform by visionK
        var kTimesTransform = visionK.times(VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        Transform2d scaledTransform = new Transform2d(
                kTimesTransform.get(0, 0),
                kTimesTransform.get(1, 0),
                Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

        // Recalculate current estimate by applying scaled transform to old estimate
        // then replaying odometry data
        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    public void updateIntakePosition(double intakeExtensionFraction) {
        this.intakeExtensionFraction = intakeExtensionFraction;
    }

    private record OdometryObservation(
            SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

    private record VisionObservation(int camIndex, Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

    public void addOdometryObservation(
            SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {
        addOdometryMeasurement(new OdometryObservation(wheelPositions, gyroAngle, timestamp));
    }

    @Override
    public void periodic() {
        // Send RobotState Data to SmartDashboard
        robotFieldInternal.setRobotPose(estimatedPose);
        SmartDashboard.putData(ROBOT_FIELD_INTERNAL_KEY, robotFieldInternal);
        robotFieldTrue.setRobotPose(getTrueOnFieldPose());
        SmartDashboard.putData(ROBOT_FIELD_TRUE_KEY, robotFieldTrue);
    }

    @Override
    public void periodicAfterScheduler() {
        // Log things
        Logger.recordOutput(ROBOT_FIELD_INTERNAL_KEY, estimatedPose);
        Logger.recordOutput(ROBOT_FIELD_TRUE_KEY, getTrueOnFieldPose());
        if (!choreoSetpoints.getInternalBuffer().isEmpty()) {
            Logger.recordOutput(
                    CHOREO_SETPOINT_KEY,
                    choreoSetpoints.getInternalBuffer().lastEntry().getValue());
        }
    }

    public void resetPose(Pose2d pose) {
        poseBuffer.clear();
        estimatedPose = pose;
        if (Constants.currentMode == Mode.SIM) SimRobot.getInstance().resetPose(pose);
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

    @Override
    public void accept(
            int cameraIndex,
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        // flip pose to get pose relative to our alliance wall
        addVisionObservation(new VisionObservation(
                cameraIndex,
                AllianceFlipUtil.apply(visionRobotPoseMeters),
                timestampSeconds,
                visionMeasurementStdDevs));
    }

    public Pose3d[] getComponentPoses() {
        Pose3d turretPose = new Pose3d();
        Pose3d hoodPose = new Pose3d();
        Pose3d intakePose = new Pose3d();
        intakePose = intakePose.transformBy(new Transform3d(Constants.SimConstants.intakeExtendedTranslation.times(intakeExtensionFraction), new Rotation3d()));
        return new Pose3d[]{intakePose, turretPose, hoodPose};
    }
}
