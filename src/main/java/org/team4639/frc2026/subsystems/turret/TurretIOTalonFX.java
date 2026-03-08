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
import java.util.Queue;
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

  private final Queue<Double> motorPositions;
  private final Queue<Double> timestampQueue;

  public TurretIOTalonFX(PortConfiguration ports) {
    turretMotor = Phoenix6Factory.createDefaultTalon(ports.TurretMotorID);

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.CurrentLimits.SupplyCurrentLimit = 20.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 20;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kS = 1;
    config.Slot0.kV = 8;
    config.Slot0.kA = 0.75;
    config.Slot0.kP = 4;

    // applyNewGains();

    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(config));

    motorPosition = turretMotor.getPosition();
    motorVelocity = turretMotor.getVelocity();
    motorVoltage = turretMotor.getMotorVoltage();
    motorCurrent = turretMotor.getStatorCurrent();

    motorPositions = PhoenixOdometryThread.getInstance().registerSignal(motorPosition.clone());
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    /*BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            motorPosition,
            motorVelocity,
            motorVoltage,
            motorCurrent
    );*/
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretMotorConnected =
        BaseStatusSignal.refreshAll(
                motorVoltage,
                motorCurrent,
                turretMotor.getDeviceTemp(),
                motorVelocity,
                motorPosition)
            .isOK();
    inputs.motorVoltage = motorVoltage.getValueAsDouble();
    inputs.motorCurrent = motorCurrent.getValueAsDouble();
    inputs.motorTemperature = turretMotor.getDeviceTemp().getValueAsDouble();
    inputs.motorVelocity = motorVelocity.getValueAsDouble();
    inputs.motorPositionRotations = motorPosition.getValueAsDouble();

    inputs.motorPositionsRotations =
        motorPositions.stream().mapToDouble((Double value) -> value).toArray();
    inputs.motorPositionsTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();

    motorPositions.clear();
    timestampQueue.clear();
  }

  @Override
  public void setRotorRotationSetpoint(double rotation) {
    turretMotor.setControl(request.withPosition(rotation));
  }

  public void setRotorRotationSetpoint(double rotation, double velocityRPS) {
    turretMotor.setControl(request.withPosition(rotation).withVelocity(velocityRPS));
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
}
