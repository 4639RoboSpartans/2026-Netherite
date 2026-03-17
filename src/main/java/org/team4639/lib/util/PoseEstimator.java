/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.NoSuchElementException;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team4639.frc2026.subsystems.drive.Drive;

public class PoseEstimator {
  private final double poseBufferSizeSec;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer;
  private final TimeInterpolatableBuffer<Pose2d> odometryBuffer;

  private final Matrix<N3, N1> qStdDevs =
      new Matrix<>(VecBuilder.fill(0.000009, 0.000009, 0.000004));

  @AutoLogOutput @Getter private Pose2d odometryPose = Pose2d.kZero;

  @AutoLogOutput @Getter private Pose2d estimatedPose = Pose2d.kZero;

  /** Assume gyro starts at zero. */
  private Rotation2d gyroOffset = Rotation2d.kZero;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(Drive.getModuleTranslations());

  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  public PoseEstimator(double poseBufferSizeSec) {
    this.poseBufferSizeSec = poseBufferSizeSec;

    poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
    odometryBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
  }

  public void addVisionObservation(
      int camIndex, Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > timestamp) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    var sample = poseBuffer.getSample(timestamp);
    if (sample.isEmpty()) return;

    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = stdDevs.get(i, 0) * stdDevs.get(i, 0);
    }
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }

    Transform2d transform = new Transform2d(estimateAtTime, visionPose);
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
  }

  public void addOdometryMeasurement(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, wheelPositions);
    lastWheelPositions = wheelPositions;

    if (odometryBuffer.getInternalBuffer().isEmpty())
      odometryBuffer.addSample(timestamp, Pose2d.kZero);
    else
      odometryBuffer.addSample(
          timestamp,
          odometryBuffer
              .getInternalBuffer()
              .get(odometryBuffer.getInternalBuffer().lastKey())
              .exp(twist));

    Pose2d lastOdometryPose = odometryPose;
    odometryPose = odometryPose.exp(twist);
    gyroAngle.ifPresent(
        angle -> {
          odometryPose = new Pose2d(odometryPose.getTranslation(), angle.plus(gyroOffset));
        });
    poseBuffer.addSample(timestamp, odometryPose);
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);
  }

  public void resetPose(Pose2d pose) {
    poseBuffer.clear();
    estimatedPose = pose;
  }
}
