/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team4639.frc2026.util.PhoenixUtil;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;

public class IntakeExtensionIOTalonFX implements IntakeExtensionIO {
    private final TalonFX extensionMotor;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final VoltageOut request = new VoltageOut(0);

    public IntakeExtensionIOTalonFX(PortConfiguration ports) {
        extensionMotor = Phoenix6Factory.createDefaultTalon(ports.intakeExtensionMotorID, false);

        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        PhoenixUtil.tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(config));
    }

    @Override
    public void updateInputs(IntakeExtensionIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                extensionMotor.getMotorVoltage(),
                extensionMotor.getStatorCurrent(),
                extensionMotor.getDeviceTemp(),
                extensionMotor.getVelocity()
        ).isOK();
        inputs.voltage = extensionMotor.getMotorVoltage().getValueAsDouble();
        inputs.current = extensionMotor.getStatorCurrent().getValueAsDouble();
        inputs.temperature = extensionMotor.getDeviceTemp().getValueAsDouble();
        inputs.velocity = extensionMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setVoltage(double appliedVoltage) {
        extensionMotor.setControl(request.withOutput(appliedVoltage));
    }

    @Override
    public void stop() {
        extensionMotor.stopMotor();
    }
}
