/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    default void setVoltage(double appliedVolts) {}

    default void setRPM(double targetRPM) {}

    default void updateInputs(ShooterIOInputs inputs) {}

    default void applyNewGains() {}

    @AutoLog
    class ShooterIOInputs {
        public boolean rightConnected = true;
        public boolean leftConnected = true;
        public double leftVoltage;
        public double rightVoltage;
        public double leftCurrent;
        public double rightCurrent;
        public double leftTemperature;
        public double rightTemperature;
        public double leftRPM;
        public double rightRPM;
        public double leftRotations;
        public double rightRotations;
    }
}
