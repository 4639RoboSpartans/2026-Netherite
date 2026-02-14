/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeExtensionIO {

    default void setVoltage(double appliedVoltage) {}

    default void stop() {}

    default void updateInputs(IntakeExtensionIOInputs inputs) {}

    @AutoLog
    class IntakeExtensionIOInputs {
        public boolean connected  = true;
        public double voltage;
        public double current;
        public double temperature;
        public double velocity;
    }
}
