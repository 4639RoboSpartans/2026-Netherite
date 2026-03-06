/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import static edu.wpi.first.units.Units.*;

public class PassingLookupTable {
    private static final InterpolatingDoubleTreeMap passingDistanceToShooterRPM = InterpolatingDoubleTreeMap.ofEntries(
    );

    private static final InterpolatingDoubleTreeMap passingDistanceToHoodRotations = InterpolatingDoubleTreeMap.ofEntries();

    public static ScoringState getPassingState(Pose2d turretPose, double passingX){
        var distance = turretPose.getX() - passingX;

        return new ScoringState(
                Rotations.per(Minute).of(passingDistanceToShooterRPM.get(distance)),
                Rotations.of(passingDistanceToHoodRotations.get(distance)),
                Degrees.of(180)
        );
    }
}
