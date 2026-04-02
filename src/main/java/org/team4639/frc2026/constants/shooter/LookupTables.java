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
import org.littletonrobotics.junction.AutoLogOutput;
import org.team4639.frc2026.Constants;
import org.team4639.lib.util.geometry.GeomUtil;

public class LookupTables {
  private static final double PHASE_DELAY = 0.0;
  private static final double TOF_FUDGE = 1.00;

  public static final double MIN_RPM = 2320.0;
  public static final double MAX_RPM = 3765.0;

  @AutoLogOutput(key = "RPM Fudge Factor")
  public static double fudge = 0.85;

  public static boolean overrideToDistance = false;
  public static double overrideDistance = 0;

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
          new AbstractMap.SimpleImmutableEntry<>(4.33, 600.),
          new AbstractMap.SimpleImmutableEntry<>(5.20, 600.),
          new AbstractMap.SimpleImmutableEntry<>(6.47, 1000.),
          new AbstractMap.SimpleImmutableEntry<>(7.36, 1300.),
          new AbstractMap.SimpleImmutableEntry<>(8.9, 1600.),
          new AbstractMap.SimpleImmutableEntry<>(11.5, 2800.));

  public static final InterpolatingDoubleTreeMap passingDistanceToHoodDegrees =
      InterpolatingDoubleTreeMap.ofEntries(
          new AbstractMap.SimpleImmutableEntry<>(4.33, 20.),
          new AbstractMap.SimpleImmutableEntry<>(5.20, 27.),
          new AbstractMap.SimpleImmutableEntry<>(6.47, 34.),
          new AbstractMap.SimpleImmutableEntry<>(7.36, 37.),
          new AbstractMap.SimpleImmutableEntry<>(8.9, 40.),
          new AbstractMap.SimpleImmutableEntry<>(11.5, 45.));

  public static final InterpolatingDoubleTreeMap passingDistanceToTOF =
      InterpolatingDoubleTreeMap.ofEntries(
          new AbstractMap.SimpleImmutableEntry<>(4.33, 0.5),
          new AbstractMap.SimpleImmutableEntry<>(5.20, 0.6),
          new AbstractMap.SimpleImmutableEntry<>(6.47, 0.7),
          new AbstractMap.SimpleImmutableEntry<>(7.36, 0.9),
          new AbstractMap.SimpleImmutableEntry<>(8.9, 1.2),
          new AbstractMap.SimpleImmutableEntry<>(11.5, 1.5));

  public static ScoringState getScoringState(
      Pose2d currentRobotPose, ChassisSpeeds fieldRelativeChassisSpeeds, Translation2d targetPose) {

    if (overrideToDistance) {
      return new ScoringState(
          scoringDistanceToRPM.get(overrideDistance),
          scoringDistanceToHoodDegrees.get(overrideDistance),
          0.5);
    }

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
    shooterRPM *= shooterRPM > 3400 ? Math.pow(1.01, fudge - 0.1) : Math.pow(1.01, fudge);
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
        passingDistanceToRPM.get(lookaheadTurretToTargetDistance) * Math.pow(1.01, fudge);
    double hoodDegrees = passingDistanceToHoodDegrees.get(lookaheadTurretToTargetDistance);

    return new ScoringState(shooterRPM, hoodDegrees, turretRotations);
  }
}
