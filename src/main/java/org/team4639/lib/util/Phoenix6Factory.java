/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import org.team4639.frc2026.util.CanDeviceId;
import org.team4639.frc2026.util.PhoenixUtil;

/**
 * Creates FXTalons objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the application.
 */
public class Phoenix6Factory {

    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
    public static final InvertedValue INVERT_VALUE = InvertedValue.CounterClockwise_Positive;
    public static final double NEUTRAL_DEADBAND = 0.04;

    // create a TalonFX with the default (out of the box) configuration
    public static TalonFX createDefaultTalon(CanDeviceId id) {
        return createDefaultTalon(id, true);
    }

    public static TalonFX createDefaultTalon(CanDeviceId id, boolean triggerConfig) {
        var talon = createTalon(id);
        if (triggerConfig) {
            PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(getTalonFXDefaultConfig()));
        }
        return talon;
    }

    public static TalonFXConfiguration getTalonFXDefaultConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NEUTRAL_MODE;
        config.MotorOutput.Inverted = INVERT_VALUE;
        config.MotorOutput.DutyCycleNeutralDeadband = NEUTRAL_DEADBAND;
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = -1.0;

        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.FeedbackRotorOffset = 0;
        config.Feedback.SensorToMechanismRatio = 1;

        config.HardwareLimitSwitch.ForwardLimitEnable = false;
        config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        config.HardwareLimitSwitch.ReverseLimitEnable = false;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
        config.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        config.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

        config.Audio.BeepOnBoot = true;

        return config;
    }

    private static TalonFX createTalon(CanDeviceId id) {
        TalonFX talon = new TalonFX(id.getDeviceNumber(), id.getBus());
        talon.clearStickyFaults();

        return talon;
    }

    public static CANcoder createCANcoder(CanDeviceId id) {
        CANcoder cancoder = new CANcoder(id.getDeviceNumber(), id.getBus());
        cancoder.clearStickyFaults();

        return cancoder;
    }
}
