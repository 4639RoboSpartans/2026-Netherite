/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.spindexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class SpindexerIOTalonFX implements SpindexerIO {
    private final TalonFX spindexerMotor;

    private final VoltageOut voltageControl = new VoltageOut(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    public SpindexerIOTalonFX(PortConfiguration ports) {
        spindexerMotor = Phoenix6Factory.createDefaultTalon(ports.KickerMotorID, false);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;

        PhoenixUtil.tryUntilOk(5, () -> spindexerMotor.getConfigurator().apply(config));
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                spindexerMotor.getMotorVoltage(),
                spindexerMotor.getStatorCurrent(),
                spindexerMotor.getVelocity(),
                spindexerMotor.getDeviceTemp()
        ).isOK();
        inputs.motorVoltage = spindexerMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorCurrent = spindexerMotor.getStatorCurrent().getValueAsDouble();
        inputs.motorVelocity = spindexerMotor.getVelocity().getValueAsDouble();
        inputs.motorTemperature = spindexerMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setVoltage(double appliedVoltage)  {
        spindexerMotor.setControl(voltageControl.withOutput(appliedVoltage));
    }

    @Override
    public void setRotorVelocityRPM(double targetVelocity) {
        spindexerMotor.setControl(velocityControl.withVelocity(targetVelocity / 60));
    }
}
