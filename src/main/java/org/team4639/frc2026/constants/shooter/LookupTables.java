/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.AbstractMap;
import org.team4639.frc2026.Constants;
import org.team4639.lib.util.geometry.GeomUtil;

public class LookupTables {
  private static final double PHASE_DELAY = 0.03;

  public static final InterpolatingDoubleTreeMap scoringDistanceToRPM =
      InterpolatingDoubleTreeMap.ofEntries(
          new AbstractMap.SimpleImmutableEntry<>(1.87, 2320.0),
          new AbstractMap.SimpleImmutableEntry<>(2.20, 2520.0),
          new AbstractMap.SimpleImmutableEntry<>(2.44, 2690.0),
          new AbstractMap.SimpleImmutableEntry<>(2.90, 2825.0),
          new AbstractMap.SimpleImmutableEntry<>(3.20, 2930.0),
          new AbstractMap.SimpleImmutableEntry<>(3.50, 3015.0),
          new AbstractMap.SimpleImmutableEntry<>(3.80, 3100.0),
          new AbstractMap.SimpleImmutableEntry<>(4.10, 3260.0),
          new AbstractMap.SimpleImmutableEntry<>(4.41, 3370.0),
          new AbstractMap.SimpleImmutableEntry<>(4.77, 3465.0),
          new AbstractMap.SimpleImmutableEntry<>(4.90, 3635.0),
          new AbstractMap.SimpleImmutableEntry<>(5.20, 3765.0));

  public static final InterpolatingDoubleTreeMap scoringDistanceToHoodDegrees =
      InterpolatingDoubleTreeMap.ofEntries(
          new AbstractMap.SimpleImmutableEntry<>(1.97, 20.0),
          new AbstractMap.SimpleImmutableEntry<>(2.15, 20.0),
          new AbstractMap.SimpleImmutableEntry<>(2.29, 20.0),
          new AbstractMap.SimpleImmutableEntry<>(2.47, 20.0),
          new AbstractMap.SimpleImmutableEntry<>(2.75, 20.0),
          new AbstractMap.SimpleImmutableEntry<>(2.97, 20.0),
          new AbstractMap.SimpleImmutableEntry<>(3.33, 20.0),
          new AbstractMap.SimpleImmutableEntry<>(3.66, 20.0),
          new AbstractMap.SimpleImmutableEntry<>(4.0, 20.0),
          new AbstractMap.SimpleImmutableEntry<>(4.25, 20.0),
          new AbstractMap.SimpleImmutableEntry<>(4.43, 20.0),
          new AbstractMap.SimpleImmutableEntry<>(4.69, 20.0),
          new AbstractMap.SimpleImmutableEntry<>(4.90, 20.0));

  public static final InterpolatingDoubleTreeMap scoringDistanceToTOF =
      InterpolatingDoubleTreeMap.ofEntries(
          new AbstractMap.SimpleImmutableEntry<>(2.47, 0.9527272727),
          new AbstractMap.SimpleImmutableEntry<>(2.47, 0.9927272727),
          new AbstractMap.SimpleImmutableEntry<>(2.7, 1.016),
          new AbstractMap.SimpleImmutableEntry<>(2.97, 1.089411765),
          new AbstractMap.SimpleImmutableEntry<>(3.3, 1.1336666667),
          new AbstractMap.SimpleImmutableEntry<>(3.66, 1.271052632),
          new AbstractMap.SimpleImmutableEntry<>(4.0, 1.351052632),
          new AbstractMap.SimpleImmutableEntry<>(4.25, 1.169),
          new AbstractMap.SimpleImmutableEntry<>(4.45, 1.171428571),
          new AbstractMap.SimpleImmutableEntry<>(4.7, 1.2146666667),
          new AbstractMap.SimpleImmutableEntry<>(4.9, 1.147368421),
          new AbstractMap.SimpleImmutableEntry<>(5.2, 1.237368421));

  public static final InterpolatingDoubleTreeMap passingDistanceToRPM = scoringDistanceToRPM;
  public static final InterpolatingDoubleTreeMap passingDistanceToHoodDegrees =
      scoringDistanceToHoodDegrees;
  public static final InterpolatingDoubleTreeMap passingDistanceToTOF = scoringDistanceToTOF;

  public static ScoringState getScoringState(
      Pose2d currentRobotPose, ChassisSpeeds fieldRelativeChassisSpeeds, Translation2d targetPose) {
    Pose2d nextEstimatedPose =
        currentRobotPose.exp(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRelativeChassisSpeeds, currentRobotPose.getRotation())
                .toTwist2d(PHASE_DELAY));
    ChassisSpeeds turretVelocity =
        GeomUtil.transformVelocity(
            fieldRelativeChassisSpeeds,
            Constants.SimConstants.originToTurretRotation.toTranslation2d(),
            currentRobotPose.getRotation());
    Pose2d turretPosition =
        nextEstimatedPose.transformBy(
            new Transform2d(
                Constants.SimConstants.originToTurretRotation.toTranslation2d(),
                Rotation2d.k180deg));

    double distanceMeters = nextEstimatedPose.getTranslation().getDistance(targetPose);
    double TOF = scoringDistanceToTOF.get(distanceMeters);

    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = distanceMeters;

    for (int i = 0; i < 20; i++) {
      TOF = scoringDistanceToTOF.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocity.vxMetersPerSecond * TOF;
      double offsetY = turretVelocity.vyMetersPerSecond * TOF;

      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = targetPose.getDistance(lookaheadPose.getTranslation());
    }

    double turretRotations =
        MathUtil.inputModulus(
            targetPose.minus(lookaheadPose.getTranslation()).getAngle().getRotations(), 0, 1);
    double shooterRPM = scoringDistanceToRPM.get(lookaheadTurretToTargetDistance);
    double hoodDegrees = scoringDistanceToHoodDegrees.get(lookaheadTurretToTargetDistance);

    return new ScoringState(shooterRPM, hoodDegrees, turretRotations);
  }

  public static ScoringState getPassingState(
      Pose2d currentRobotPose, ChassisSpeeds fieldRelativeChassisSpeeds, Translation2d targetPose) {
    Pose2d nextEstimatedPose =
        currentRobotPose.exp(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRelativeChassisSpeeds, currentRobotPose.getRotation())
                .toTwist2d(PHASE_DELAY));
    ChassisSpeeds turretVelocity =
        GeomUtil.transformVelocity(
            fieldRelativeChassisSpeeds,
            Constants.SimConstants.originToTurretRotation.toTranslation2d(),
            currentRobotPose.getRotation());
    Pose2d turretPosition =
        nextEstimatedPose.transformBy(
            new Transform2d(
                Constants.SimConstants.originToTurretRotation.toTranslation2d(),
                Rotation2d.k180deg));

    double distanceMeters = nextEstimatedPose.getTranslation().getDistance(targetPose);
    double TOF = scoringDistanceToTOF.get(distanceMeters);

    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = distanceMeters;

    for (int i = 0; i < 20; i++) {
      TOF = passingDistanceToTOF.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocity.vxMetersPerSecond * TOF;
      double offsetY = turretVelocity.vyMetersPerSecond * TOF;

      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      lookaheadTurretToTargetDistance = targetPose.getDistance(lookaheadPose.getTranslation());
    }

    double turretRotations =
        MathUtil.inputModulus(
            targetPose.minus(lookaheadPose.getTranslation()).getAngle().getRotations(), 0, 1);
    double shooterRPM = passingDistanceToRPM.get(lookaheadTurretToTargetDistance);
    double hoodDegrees = passingDistanceToHoodDegrees.get(lookaheadTurretToTargetDistance);

    return new ScoringState(shooterRPM, hoodDegrees, turretRotations);
  }
}
