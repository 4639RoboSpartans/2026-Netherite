/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {

    default void setSurfaceVelocityFeetPerSecond(double targetVelocity) {}

    default void setVoltage(double volts) {}

    default void updateInputs(IntakeRollerIOInputs inputs) {}

    /** newGains in order: kP, kI, kD, kS, kV, kA */
    default void applyNewGains(double[] newGains) {}

    @AutoLog
    class IntakeRollerIOInputs {
        public boolean connected = true;
        public double volts;
        public double amps;
        public double celsius;
        public double rotationsPerSecond;
        public double rotations;
    }
}
