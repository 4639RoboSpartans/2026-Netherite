/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.team4639.frc2026.util.PhoenixUtil;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;

import static org.team4639.frc2026.subsystems.hood.Constants.*;

public class HoodIOTalonFX implements HoodIO {
    private final TalonFX hoodMotor;
    private final CANcoder hoodEncoder;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final PositionVoltage request = new PositionVoltage(0);

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorCurrent;

    public HoodIOTalonFX(PortConfiguration ports) {
        hoodMotor = Phoenix6Factory.createDefaultTalon(ports.HoodMotorID);
        hoodEncoder = Phoenix6Factory.createCANcoder(ports.HoodEncoderID);

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.Audio.BeepOnConfig = false;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.ClosedLoopGeneral.ContinuousWrap = true;

        applyNewGains();

        hoodPosition = hoodEncoder.getPosition();
        hoodVelocity = hoodEncoder.getVelocity();
        motorVoltage = hoodMotor.getMotorVoltage();
        motorCurrent = hoodMotor.getStatorCurrent();
    }

    @Override
    public void setSetpointDegrees(double setpointDegrees) {
        double rotation = (setpointDegrees - HOOD_MIN_ANGLE_DEGREES) * ENCODER_ROTATIONS_PER_DEGREE + HOOD_ENCODER_MIN_ROTATION;
        rotation = MathUtil.clamp(rotation, HOOD_ENCODER_MIN_ROTATION, HOOD_ENCODER_MAX_ROTATION);
        hoodMotor.setControl(request.withPosition(rotation));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodMotorConnected = BaseStatusSignal.refreshAll(
                motorVoltage,
                motorCurrent,
                hoodMotor.getDeviceTemp(),
                hoodVelocity,
                hoodPosition
        ).isOK();
        inputs.pivotVoltage = motorVoltage.getValueAsDouble();
        inputs.pivotCurrent = motorCurrent.getValueAsDouble();
        inputs.pivotTemperature = hoodMotor.getDeviceTemp().getValueAsDouble();
        inputs.pivotPositionDegrees = (hoodPosition.getValueAsDouble() - HOOD_ENCODER_MIN_ROTATION) / ENCODER_ROTATIONS_PER_DEGREE + HOOD_MIN_ANGLE_DEGREES;
        inputs.pivotVelocityDegrees = hoodVelocity.getValueAsDouble() / ENCODER_ROTATIONS_PER_DEGREE;
    }

    @Override
    public void setVoltage(double volts){
        hoodMotor.setVoltage(volts);
    }

    public void updateGains() {
        config.Slot0.kP = PIDs.hoodKp.get();
        config.Slot0.kI = PIDs.hoodKi.get();
        config.Slot0.kD = PIDs.hoodKd.get();
        config.Slot0.kS = PIDs.hoodKs.get();
        config.Slot0.kV = PIDs.hoodKv.get();
        config.Slot0.kA = PIDs.hoodKa.get();
    }

    @Override
    public void applyNewGains() {
        updateGains();
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config));
    }
}
