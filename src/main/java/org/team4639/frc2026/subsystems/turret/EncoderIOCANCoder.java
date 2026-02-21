/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.team4639.frc2026.util.CanDeviceId;

public class EncoderIOCANCoder implements EncoderIO {
    private final CANcoder encoder;

    public EncoderIOCANCoder(CanDeviceId canDeviceId, double offsetRotations, boolean inverted) {
        encoder = new CANcoder(canDeviceId.getDeviceNumber(), canDeviceId.getBus());
        var cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = offsetRotations;
        cancoderConfigs.MagnetSensor.SensorDirection =
                inverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        var response = encoder.getConfigurator().apply(cancoderConfigs);
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.positionRotations = encoder.getAbsolutePosition().getValueAsDouble();
    }
}
