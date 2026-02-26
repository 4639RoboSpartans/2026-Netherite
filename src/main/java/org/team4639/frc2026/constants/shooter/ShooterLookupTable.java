/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.shooter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

/**
 * @param scoringDistanceToRPM       meters -> RPM
 * @param scoringDistanceToHoodAngle meters -> encoder angle
 * @param scoringDistanceToTOF       meters -> seconds
 */
public record ShooterLookupTable(InterpolatingDoubleTreeMap scoringDistanceToRPM,
                                 InterpolatingDoubleTreeMap scoringDistanceToHoodAngle,
                                 InterpolatingDoubleTreeMap scoringDistanceToTOF) {

    private static final Vector<N2> i_hat = VecBuilder.fill(1, 0);

    public ScoringState calculateShooterStateStationary(Pose2d turretPose, Translation2d hubTranslation) {
        Translation2d robotTranslation = turretPose.getTranslation();
        Rotation2d robotRotation = turretPose.getRotation();
        Translation2d robotToHubTranslation = hubTranslation.minus(robotTranslation);
        Rotation2d fieldRelativeHubDirection = robotToHubTranslation.getAngle();
        double distanceMeters = robotToHubTranslation.getNorm();
        SmartDashboard.putNumber("Turret To Hub", distanceMeters);
        double shooterRPM = scoringDistanceToRPM.get(distanceMeters);
        double hoodAngle = scoringDistanceToHoodAngle.get(distanceMeters);
        return new ScoringState(Rotations.per(Minute).of(shooterRPM), Rotations.of(hoodAngle), fieldRelativeHubDirection.getMeasure());
    }

    public ScoringState convergeShooterStateSOTFTurret(Pose2d turretPose, Translation2d hubTranslation, Translation2d turretVelocity, int maxIterations) {
        Translation2d turretTranslation = turretPose.getTranslation();
        Rotation2d robotRotation = turretPose.getRotation();
        Translation2d turretDisplacement = new Translation2d(0, 0);
        for (int i = 0; i < maxIterations; i++) {
            Translation2d robotTranslationAfterDisplacement = turretTranslation.plus(turretDisplacement);
            Translation2d robotDisplacedToHubTranslation = hubTranslation.minus(robotTranslationAfterDisplacement);
            double newDistance = robotDisplacedToHubTranslation.getNorm();
            double newTOF = scoringDistanceToTOF.get(newDistance);
            turretDisplacement = turretVelocity.times(newTOF);
        }
        Translation2d finalRobotToHubTranslation = hubTranslation.minus(turretTranslation.plus(turretDisplacement));
        double distanceMeters = finalRobotToHubTranslation.getNorm();
        double shooterRPM = scoringDistanceToRPM.get(distanceMeters);
        double hoodAngle = scoringDistanceToHoodAngle.get(distanceMeters);
        return new ScoringState(Rotations.per(Minute).of(shooterRPM), Rotations.of(hoodAngle), finalRobotToHubTranslation.getAngle().getMeasure());
    }
}
