/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.team4639.frc2026.util.PhoenixUtil;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX turretMotor;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final PositionVoltage request = new PositionVoltage(0);

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorCurrent;

    public TurretIOTalonFX(PortConfiguration ports) {
        turretMotor = Phoenix6Factory.createDefaultTalon(ports.TurretMotorID);

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kS = 0.4703;
        config.Slot0.kV = 6.0897;
        config.Slot0.kA = 0.54764;
        config.Slot0.kP = 87.009;

        applyNewGains();

        motorPosition = turretMotor.getPosition();
        motorVelocity = turretMotor.getVelocity();
        motorVoltage = turretMotor.getMotorVoltage();
        motorCurrent = turretMotor.getStatorCurrent();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretMotorConnected = BaseStatusSignal.refreshAll(
                motorVoltage,
                motorCurrent,
                turretMotor.getDeviceTemp(),
                motorVelocity,
                motorPosition
        ).isOK();
        inputs.motorVoltage = motorVoltage.getValueAsDouble();
        inputs.motorCurrent = motorCurrent.getValueAsDouble();
        inputs.motorTemperature = turretMotor.getDeviceTemp().getValueAsDouble();
        inputs.motorVelocity = motorVelocity.getValueAsDouble();
        inputs.motorPositionRotations = motorPosition.getValueAsDouble();
    }

    @Override
    public void setRotorRotationSetpoint(double rotation) {
        turretMotor.setControl(request.withPosition(rotation));
    }

    public void updateGains() {
        config.Slot0.kP = PIDs.turretKp.get();
        config.Slot0.kI = PIDs.turretKi.get();
        config.Slot0.kD = PIDs.turretKd.get();
        config.Slot0.kS = PIDs.turretKs.get();
        config.Slot0.kV = PIDs.turretKv.get();
        config.Slot0.kA = PIDs.turretKa.get();
    }

    @Override
    public void applyNewGains() {
        updateGains();
        PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(config));
    }

    @Override
    public void setVoltage(double volts){
        turretMotor.setVoltage(volts);
    }
}
