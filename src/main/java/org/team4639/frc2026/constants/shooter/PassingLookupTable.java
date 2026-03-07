/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.team4639.frc2026.FieldConstants;

import java.util.AbstractMap;

import static edu.wpi.first.units.Units.*;

public class PassingLookupTable {
    private static final InterpolatingDoubleTreeMap passingDistanceToShooterRPM = InterpolatingDoubleTreeMap.ofEntries(
            new AbstractMap.SimpleImmutableEntry<>(FieldConstants.LinesVertical.allianceZone/2, 1000.0),
            new AbstractMap.SimpleImmutableEntry<>(FieldConstants.LinesVertical.oppAllianceZone, 6000.0)
    );

    private static final InterpolatingDoubleTreeMap passingDistanceToHoodRotations = InterpolatingDoubleTreeMap.ofEntries(
            new AbstractMap.SimpleImmutableEntry<>(FieldConstants.LinesVertical.allianceZone/2, 50.0 / 360.0),
            new AbstractMap.SimpleImmutableEntry<>(FieldConstants.LinesVertical.oppAllianceZone, 30.0 / 360.0)
    );

    public static ScoringState getPassingState(Pose2d turretPose, double passingX){
        var distance = turretPose.getX() - passingX;

        return new ScoringState(
                Rotations.per(Minute).of(passingDistanceToShooterRPM.get(distance)),
                Rotations.of(passingDistanceToHoodRotations.get(distance)),
                Degrees.of(180)
        );
    }
}
