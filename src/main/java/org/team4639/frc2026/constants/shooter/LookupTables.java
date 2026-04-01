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
import org.team4639.lib.util.LoggedTunableNumber;
import org.team4639.lib.util.geometry.GeomUtil;

public class LookupTables {
  private static final double PHASE_DELAY = 0.0;
  private static final double TOF_FUDGE = 1.00;

  public static final double MIN_RPM = 2320.0;
  public static final double MAX_RPM = 3765.0;

  public static final LoggedTunableNumber fudge =
      new LoggedTunableNumber("RPM Fudge").initDefault(0.85);

  public static final InterpolatingDoubleTreeMap scoringDistanceToRPM =
      InterpolatingDoubleTreeMap.ofEntries(
          new AbstractMap.SimpleImmutableEntry<>(1.87, MIN_RPM),
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
          new AbstractMap.SimpleImmutableEntry<>(5.20, MAX_RPM));

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
          new AbstractMap.SimpleImmutableEntry<>(1.87, 0.8814285714 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(2.2, 0.9725 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(2.5, 1.0175 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(2.9, 1.035 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(3.2, 1.061428571 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(3.5, 1.176666667 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(3.8, 1.192 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(4.1, 1.238571429 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(4.4, 1.3425 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(4.77, 1.42 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(5.2, 1.4725 * TOF_FUDGE));

  public static final InterpolatingDoubleTreeMap passingDistanceToRPM =
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
          new AbstractMap.SimpleImmutableEntry<>(5.20, 3765.0),
          // New Setpoints
          new AbstractMap.SimpleImmutableEntry<>(5.5, 4005.3875),
          new AbstractMap.SimpleImmutableEntry<>(5.8, 4280.9288),
          new AbstractMap.SimpleImmutableEntry<>(6.1, 4611.5969),
          new AbstractMap.SimpleImmutableEntry<>(6.5, 5150.8125),
          new AbstractMap.SimpleImmutableEntry<>(6.8, 5638.3968),
          new AbstractMap.SimpleImmutableEntry<>(7.1, 6205.3539),
          new AbstractMap.SimpleImmutableEntry<>(7.4, 6858.9576));

  public static final InterpolatingDoubleTreeMap passingDistanceToHoodDegrees =
      InterpolatingDoubleTreeMap.ofEntries(
          new AbstractMap.SimpleImmutableEntry<>(0.0, 35.0),
          new AbstractMap.SimpleImmutableEntry<>(6.0, 35.0),
          new AbstractMap.SimpleImmutableEntry<>(6.00001, 50.0),
          new AbstractMap.SimpleImmutableEntry<>(20.0, 50.0));

  public static final InterpolatingDoubleTreeMap passingDistanceToTOF =
      InterpolatingDoubleTreeMap.ofEntries(
          new AbstractMap.SimpleImmutableEntry<>(1.87, 0.8814285714 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(2.2, 0.9725 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(2.5, 1.0175 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(2.9, 1.035 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(3.2, 1.061428571 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(3.5, 1.176666667 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(3.8, 1.192 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(4.1, 1.238571429 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(4.4, 1.3425 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(4.77, 1.42 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(5.2, 1.4725 * TOF_FUDGE),
          // New Setpoints
          new AbstractMap.SimpleImmutableEntry<>(5.5, 1.55855625 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(5.8, 1.629534 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(6.1, 1.70293275 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(6.5, 1.80449375 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(6.8, 1.883384 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(7.1, 1.96456025 * TOF_FUDGE),
          new AbstractMap.SimpleImmutableEntry<>(7.4, 2.047982));

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
    double shooterRPM =
        scoringDistanceToRPM.get(lookaheadTurretToTargetDistance) * Math.pow(1.01, fudge.get());
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
    double shooterRPM =
        passingDistanceToRPM.get(lookaheadTurretToTargetDistance) * Math.pow(1.01, fudge.get());
    double hoodDegrees = passingDistanceToHoodDegrees.get(lookaheadTurretToTargetDistance);

    return new ScoringState(shooterRPM, hoodDegrees, turretRotations);
  }
}
