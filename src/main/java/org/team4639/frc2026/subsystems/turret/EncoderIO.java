/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {
    default void updateInputs(EncoderIOInputs inputs) {}

    @AutoLog
    class EncoderIOInputs {
        public double positionRotations = 0.0;
    }
}
