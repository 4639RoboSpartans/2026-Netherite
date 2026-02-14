/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team4639.frc2026.util.PhoenixUtil;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;

import static edu.wpi.first.units.Units.*;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
    private final TalonFX rollerMotor;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final VelocityVoltage request = new VelocityVoltage(0);

    private final double ROTOR_TO_ROLLER_REDUCTION = 24.0 / 12.0;
    private final double ROLLER_RADIUS = 1.0;

    public IntakeRollerIOTalonFX(PortConfiguration ports) {
        rollerMotor = Phoenix6Factory.createDefaultTalon(ports.intakeRollersMotorID, false);

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(config));
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                rollerMotor.getMotorVoltage(),
                rollerMotor.getStatorCurrent(),
                rollerMotor.getDeviceTemp(),
                rollerMotor.getVelocity()
        ).isOK();
        inputs.voltage = rollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.current = rollerMotor.getStatorCurrent().getValueAsDouble();
        inputs.temperature = rollerMotor.getDeviceTemp().getValueAsDouble();
        inputs.velocity = rollerMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setSurfaceVelocityFeetPerSecond(double targetVelocity) {
        double targetSurfaceVelocityInchPerSecond = FeetPerSecond.of(targetVelocity).in(InchesPerSecond);
        double targetRollerVelocityRadiansPerSecond = targetSurfaceVelocityInchPerSecond / ROLLER_RADIUS;
        double targetRotorVelocityRadiansPerSecond = targetRollerVelocityRadiansPerSecond * ROTOR_TO_ROLLER_REDUCTION;
        double targetRotorVelocityRotationsPerSecond = RadiansPerSecond.of(targetRotorVelocityRadiansPerSecond).in(RotationsPerSecond);
        rollerMotor.setControl(request.withVelocity(targetRotorVelocityRotationsPerSecond));
    }
}
