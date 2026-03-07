/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.shooter;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;


/**
 * @param shooterRPM shooter flywheel RPM
 * @param hoodAngle shooter angle relative to horizontal
 * @param turretAngle turret angle field-relative
 */
public record ScoringState(AngularVelocity shooterRPM, Angle hoodAngle, Angle turretAngle) {
    @Override
    public String toString() {
        return "RPM: " + shooterRPM.in(Rotations.per(Minute)) + " Hood Angle: " + hoodAngle.in(Rotations) + " turretAngle: "  + turretAngle.in(Rotations);
    }

    public ScoringState replace(AngularVelocity shooterRPM, Angle hoodAngle, Angle turretAngle) {
        var newShooterRPM = this.shooterRPM;
        var newHoodAngle = this.hoodAngle;
        var newTurretAngle = this.turretAngle;

        if (shooterRPM != null) {
            newShooterRPM = shooterRPM;
        }

        if (hoodAngle != null) {
            newHoodAngle = hoodAngle;
        }

        if (turretAngle != null) {
            newTurretAngle = turretAngle;
        }

        return new ScoringState(newShooterRPM, newHoodAngle, newTurretAngle);
    }

    public ScoringState times(double rpmScalar, double hoodAngleScalar, double turretAngleScalar){
        return replace(shooterRPM.times(rpmScalar), hoodAngle.times(hoodAngleScalar), turretAngle.times(turretAngleScalar));
    }
}
