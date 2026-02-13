/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

    default void setVoltage(double appliedVolts) {}

    default void setRotorVelocityRPM(double targetVelocity) {}

    default void updateInputs(SpindexerIOInputs inputs) {}

    default void applyNewGains() {}

    @AutoLog
    class SpindexerIOInputs {
        public boolean motorConnected = true;
        public double motorVoltage;
        public double motorCurrent;
        public double motorVelocity;
        public double motorTemperature;
    }
}
