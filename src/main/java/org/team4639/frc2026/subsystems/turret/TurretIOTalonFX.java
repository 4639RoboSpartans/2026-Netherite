/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.LoggedTunableNumber;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX turretMotor;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final PositionVoltage request = new PositionVoltage(0);
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    private final VoltageOut voltageOut = new VoltageOut(0).withIgnoreSoftwareLimits(true);

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorCurrent;

    private double lastVelocity = 0;

    private final LoggedTunableNumber KA = new LoggedTunableNumber("turret KA").initDefault(0.0007793003);

    public TurretIOTalonFX(PortConfiguration ports) {
        turretMotor = Phoenix6Factory.createDefaultTalon(ports.TurretMotorID);

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.CurrentLimits.SupplyCurrentLimit = 20.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 30;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        double kS = 0.1813163;
        double kV = 0.01055999667;
        double kA = 0.0007793003;

        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kP = 0.5; // mild kP, most of control here is done through velocity

        config.Slot1.kP = 15; // most aggressive for turret wraparound

        config.Slot2.kP = 4; // want to reach zero setpoint with accuracy, don't care about speed but don't
        // want
        // steady state error

        config.MotionMagic.MotionMagicCruiseVelocity = 2.0 / Constants.MOTOR_TO_TURRET_GEAR_RATIO;
        config.MotionMagic.MotionMagicAcceleration = 8.0 / Constants.MOTOR_TO_TURRET_GEAR_RATIO;

        PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(config));

        motorPosition = turretMotor.getPosition();
        motorVelocity = turretMotor.getVelocity();
        motorVoltage = turretMotor.getMotorVoltage();
        motorCurrent = turretMotor.getStatorCurrent();

        RobotState.disabled.onTrue(Commands.runOnce(
                        () -> PhoenixUtil.tryUntilOk(5, () -> turretMotor.setNeutralMode(NeutralModeValue.Coast)))
                .ignoringDisable(true));
        RobotState.disabled.onFalse(Commands.runOnce(
                        () -> PhoenixUtil.tryUntilOk(5, () -> turretMotor.setNeutralMode(NeutralModeValue.Brake)))
                .ignoringDisable(true));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                        motorVoltage, motorCurrent, turretMotor.getDeviceTemp(), motorVelocity, motorPosition)
                .isOK();
        inputs.volts = motorVoltage.getValueAsDouble();
        inputs.amps = motorCurrent.getValueAsDouble();
        inputs.celsius = turretMotor.getDeviceTemp().getValueAsDouble();
        inputs.rotationsPerSecond = motorVelocity.getValueAsDouble();
        inputs.rotations = motorPosition.getValueAsDouble();

        double timestamp = RobotController.getFPGATime() / 1e6;
        double latency = motorPosition.getTimestamp().getLatency();

        inputs.timestamp = timestamp - latency;
    }

    @Override
    public void setRotorRotationSetpoint(double rotation) {
        turretMotor.setControl(request.withPosition(rotation));
    }

    public void setRotorRotationSetpoint(double rotation, double velocityRPS) {
        turretMotor.setControl(
                request.withPosition(rotation).withVelocity(velocityRPS).withSlot(0));
    }

    public void setRotorRotationSetpointSlot1(double rotation, double velocityRPS) {
        turretMotor.setControl(request.withPosition(rotation).withSlot(1));
    }

    public void setRotorRotationSetpointSlot2(double rotation) {
        turretMotor.setControl(request.withPosition(rotation).withSlot(2).withVelocity(0));
    }

    @Override
    public void setVoltage(double volts) {
        turretMotor.setControl(voltageOut.withOutput(volts));
    }

    public void setSoftwareLimits(double forwardLimitRotorRotations, double reverseLimitRotorRotations) {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimitRotorRotations;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimitRotorRotations;

        PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(config));
    }

    public void setRotorRotations(double rotorRotations) {
        turretMotor.setPosition(rotorRotations);
    }
}
