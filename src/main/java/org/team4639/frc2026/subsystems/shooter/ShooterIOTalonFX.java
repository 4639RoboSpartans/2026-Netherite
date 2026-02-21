/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team4639.frc2026.util.PhoenixUtil;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leftShooter;
    private final TalonFX rightShooter;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final VoltageOut shooterVoltageControl = new VoltageOut(0);

    private final VelocityVoltage shooterVelocityControl = new VelocityVoltage(0);

    private final double SHOOTER_GEAR_RATIO = 1.0;

    public ShooterIOTalonFX(PortConfiguration ports) {
        leftShooter = Phoenix6Factory.createDefaultTalon(ports.shooterMotorLeftID);
        rightShooter = Phoenix6Factory.createDefaultTalon(ports.shooterMotorRightID);
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

        applyNewGains();

        leftShooter.setControl(new Follower(rightShooter.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.rightConnected = BaseStatusSignal.refreshAll(
                        rightShooter.getMotorVoltage(),
                        rightShooter.getSupplyCurrent(),
                        rightShooter.getDeviceTemp(),
                        rightShooter.getVelocity())
                .isOK();
        inputs.leftConnected = BaseStatusSignal.refreshAll(
                        leftShooter.getMotorVoltage(),
                        leftShooter.getSupplyCurrent(),
                        leftShooter.getDeviceTemp(),
                        leftShooter.getVelocity())
                .isOK();
        inputs.leftVoltage = leftShooter.getMotorVoltage().getValueAsDouble();
        inputs.rightVoltage = rightShooter.getMotorVoltage().getValueAsDouble();
        inputs.leftCurrent = leftShooter.getSupplyCurrent().getValueAsDouble();
        inputs.rightCurrent = rightShooter.getSupplyCurrent().getValueAsDouble();
        inputs.leftTemperature = leftShooter.getDeviceTemp().getValueAsDouble();
        inputs.rightTemperature = rightShooter.getDeviceTemp().getValueAsDouble();
        inputs.leftRPM = (leftShooter.getRotorVelocity().getValueAsDouble() * 60) / SHOOTER_GEAR_RATIO;
        inputs.rightRPM = (rightShooter.getRotorVelocity().getValueAsDouble() * 60) / SHOOTER_GEAR_RATIO;
        inputs.leftRotations = leftShooter.getRotorPosition().getValueAsDouble() / SHOOTER_GEAR_RATIO;
        inputs.rightRotations = rightShooter.getRotorPosition().getValueAsDouble() / SHOOTER_GEAR_RATIO;
    }

    @Override
    public void setVoltage(double appliedVolts) {
        rightShooter.setControl(shooterVoltageControl.withOutput(appliedVolts));
    }

    @Override
    public void setRPM(double targetRPM) {
        double applied = targetRPM * SHOOTER_GEAR_RATIO / 60.0;

        rightShooter.setControl(shooterVelocityControl.withVelocity(applied));
    }

    public void updateGains() {
        config.Slot0.kP = PIDs.shooterKp.get();
        config.Slot0.kI = PIDs.shooterKi.get();
        config.Slot0.kD = PIDs.shooterKd.get();
        config.Slot0.kS = PIDs.shooterKs.get();
        config.Slot0.kV = PIDs.shooterKv.get();
        config.Slot0.kA = PIDs.shooterKa.get();
    }

    @Override
    public void applyNewGains() {
        updateGains();
        PhoenixUtil.tryUntilOk(5, () -> leftShooter.getConfigurator().apply(config));
        PhoenixUtil.tryUntilOk(5, () -> rightShooter.getConfigurator().apply(config));
    }
}
