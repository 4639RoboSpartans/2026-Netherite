/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    default void setSetpointDegrees(double setpointDegrees) {}

    default void setVoltage(double volts) {}

    default void updateInputs(HoodIOInputs inputs) {}

    default void applyNewGains() {}

    @AutoLog
    class HoodIOInputs {
        public boolean hoodMotorConnected = true;
        public boolean pivotEncoderConnected = true;
        public double pivotVoltage = 0.0;
        public double pivotCurrent = 0.0;
        public double pivotTemperature = 0.0;
        public double pivotPositionDegrees = 0.0;
        public double pivotVelocityDegrees = 0.0;
    }
}
