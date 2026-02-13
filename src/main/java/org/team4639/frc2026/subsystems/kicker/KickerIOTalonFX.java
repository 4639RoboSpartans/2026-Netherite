/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.kicker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class KickerIOTalonFX implements KickerIO {
    private final TalonFX kickerMotor;

    private final VoltageOut voltageControl = new VoltageOut(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    public KickerIOTalonFX(PortConfiguration ports) {
        kickerMotor = Phoenix6Factory.createDefaultTalon(ports.KickerMotorID, false);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;

        PhoenixUtil.tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(config));
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                kickerMotor.getMotorVoltage(),
                kickerMotor.getStatorCurrent(),
                kickerMotor.getVelocity(),
                kickerMotor.getDeviceTemp()
        ).isOK();
        inputs.motorVoltage = kickerMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorCurrent = kickerMotor.getStatorCurrent().getValueAsDouble();
        inputs.motorVelocity = kickerMotor.getVelocity().getValueAsDouble();
        inputs.motorTemperature = kickerMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setVoltage(double appliedVoltage)  {
        kickerMotor.setControl(voltageControl.withOutput(appliedVoltage));
    }

    @Override
    public void setRotorVelocityRPM(double targetVelocity) {
        kickerMotor.setControl(velocityControl.withVelocity(targetVelocity / 60));
    }
}
