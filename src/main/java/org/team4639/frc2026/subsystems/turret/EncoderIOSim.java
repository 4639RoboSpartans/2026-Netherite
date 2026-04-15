/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.turret;

public class EncoderIOSim implements EncoderIO {
    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.positionRotations = 0;
    }
}
