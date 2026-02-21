/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intakeRollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {

    default void setSurfaceVelocityFeetPerSecond(double targetVelocity) {}

    default void setVoltage(double volts) {}

    default void updateInputs(IntakeRollerIOInputs inputs) {}

    /**
     * newGains in order: kP, kI, kD, kS, kV, kA
     */
    default void applyNewGains(double[] newGains) {}

    @AutoLog
    class IntakeRollerIOInputs {
        public boolean connected = true;
        public double voltage;
        public double current;
        public double temperature;
        public double velocity;
        public double position;
    }
}
