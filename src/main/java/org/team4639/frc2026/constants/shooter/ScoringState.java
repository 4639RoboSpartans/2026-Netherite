/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

public record ScoringState(AngularVelocity shooterRPM, Angle hoodAngle, Angle turretAngle) {
    @Override
    public String toString() {
        return "RPM: " + shooterRPM.in(Rotations.per(Minute)) + " Hood Angle: " + hoodAngle.in(Rotations) + " turretAngle: "  + turretAngle.in(Rotations);
    }
}
