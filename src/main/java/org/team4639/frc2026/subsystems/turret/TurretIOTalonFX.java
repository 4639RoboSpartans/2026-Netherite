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
import edu.wpi.first.wpilibj2.command.Commands;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

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
    config.CurrentLimits.SupplyCurrentLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 20;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    double kS = 0.1829948;
    double kV = 0.09422215;
    double kA = 0.0005154738;

    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kP = 0.5;

    config.Slot1.kP = 15;

    // applyNewGains();

    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(config));

    motorPosition = turretMotor.getPosition();
    motorVelocity = turretMotor.getVelocity();
    motorVoltage = turretMotor.getMotorVoltage();
    motorCurrent = turretMotor.getStatorCurrent();

    /*BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            motorPosition,
            motorVelocity,
            motorVoltage,
            motorCurrent
    );*/

    RobotState.disabled.onTrue(
        Commands.runOnce(
                () ->
                    PhoenixUtil.tryUntilOk(
                        5, () -> turretMotor.setNeutralMode(NeutralModeValue.Coast)))
            .ignoringDisable(true));
    RobotState.disabled.onFalse(
        Commands.runOnce(
                () ->
                    PhoenixUtil.tryUntilOk(
                        5, () -> turretMotor.setNeutralMode(NeutralModeValue.Brake)))
            .ignoringDisable(true));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                motorVoltage,
                motorCurrent,
                turretMotor.getDeviceTemp(),
                motorVelocity,
                motorPosition)
            .isOK();
    inputs.volts = motorVoltage.getValueAsDouble();
    inputs.amps = motorCurrent.getValueAsDouble();
    inputs.celsius = turretMotor.getDeviceTemp().getValueAsDouble();
    inputs.rotationsPerSecond = motorVelocity.getValueAsDouble();
    inputs.rotations = motorPosition.getValueAsDouble();
  }

  @Override
  public void setRotorRotationSetpoint(double rotation) {
    turretMotor.setControl(request.withPosition(rotation));
  }

  public void setRotorRotationSetpoint(double rotation, double velocityRPS) {
    turretMotor.setControl(request.withPosition(rotation).withVelocity(velocityRPS).withSlot(0));
  }

  public void setRotorRotationSetpointSlot1(double rotation, double velocityRPS) {
    turretMotor.setControl(request.withPosition(rotation).withSlot(1));
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
    // updateGains();
    // PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(config));
  }

  @Override
  public void setVoltage(double volts) {
    turretMotor.setVoltage(volts);
  }

  public void setSoftwareLimits(
      double forwardLimitRotorRotations, double reverseLimitRotorRotations) {
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimitRotorRotations;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimitRotorRotations;

    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(config));
  }
}
