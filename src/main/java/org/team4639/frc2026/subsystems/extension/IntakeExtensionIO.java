/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.extension;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtensionIO {

    default void setVoltage(double appliedVoltage) {}

    default void stop() {}

    default void updateInputs(IntakeExtensionIOInputs inputs) {}

    default void setBrakeMode(boolean brake) {}

    @AutoLog
    class IntakeExtensionIOInputs {
        public boolean connected = true;
        public double volts;
        public double amps;
        public double celsius;
        public double rotationsPerSecond;
        public double rotations;
    }
}
