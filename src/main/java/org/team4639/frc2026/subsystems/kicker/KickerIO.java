/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {

    default void setVoltage(double appliedVolts) {}

    default void setMechanismVelocityRPM(double targetVelocity) {}

    default void updateInputs(KickerIOInputs inputs) {}

    @AutoLog
    class KickerIOInputs {
        public boolean connected = true;
        public double volts;
        public double amps;
        public double rotationsPerSecond;
        public double celsius;
        public double rotations;
    }
}
