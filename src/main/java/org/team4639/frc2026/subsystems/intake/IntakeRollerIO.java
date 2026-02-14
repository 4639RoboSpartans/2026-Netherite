/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {

    default void setSurfaceVelocityFeetPerSecond(double targetVelocity) {}

    default void updateInputs(IntakeRollerIOInputs inputs) {}

    @AutoLog
    class IntakeRollerIOInputs {
        public boolean connected = true;
        public double voltage;
        public double current;
        public double temperature;
        public double velocity;
    }
}
