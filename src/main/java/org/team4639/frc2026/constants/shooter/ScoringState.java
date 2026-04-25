/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.shooter;

/**
 * @param shooterRPM
 *            shooter flywheel RPM
 * @param hoodDegrees
 *            shooter angle relative to horizontal
 * @param turretRotations
 *            turret angle field-relative
 */
public record ScoringState(double shooterRPM, double hoodDegrees, double turretRotations) {
    @Override
    public String toString() {
        return "RPM: " + shooterRPM + " Hood Degrees: " + hoodDegrees + " Hurret Rotations: " + turretRotations;
    }

    public ScoringState replace(double shooterRPM, double hoodDegrees, double turretRotations) {
        var newShooterRPM = this.shooterRPM;
        var newHoodAngle = this.hoodDegrees;
        var newTurretAngle = this.turretRotations;

        if (!Double.isNaN(shooterRPM)) {
            newShooterRPM = shooterRPM;
        }

        if (!Double.isNaN(hoodDegrees)) {
            newHoodAngle = hoodDegrees;
        }

        if (!Double.isNaN(turretRotations)) {
            newTurretAngle = turretRotations;
        }

        return new ScoringState(newShooterRPM, newHoodAngle, newTurretAngle);
    }

    public ScoringState times(double rpmScalar, double hoodAngleScalar, double turretAngleScalar) {
        return replace(
                shooterRPM * (rpmScalar), hoodDegrees * (hoodAngleScalar), turretRotations * (turretAngleScalar));
    }
}
