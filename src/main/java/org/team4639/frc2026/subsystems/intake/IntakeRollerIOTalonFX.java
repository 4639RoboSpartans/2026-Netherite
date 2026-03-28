/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX rollerMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VelocityVoltage request = new VelocityVoltage(0);

  public IntakeRollerIOTalonFX(PortConfiguration ports) {
    rollerMotor = Phoenix6Factory.createDefaultTalon(ports.IntakeRollersMotorID, false);

    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Slot0.kS = 0.25526;
    config.Slot0.kV = 0.1194121;
    config.Slot0.kA = 0.002791;

    config.Slot0.kP = 999999;
    config.MotorOutput.PeakForwardDutyCycle = 0;
    config.MotorOutput.PeakReverseDutyCycle = -1;

    config.Slot1.kS = 0.25526;
    config.Slot1.kV = 0.1194121;
    config.Slot1.kA = 0.002791;

    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(config));
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                rollerMotor.getMotorVoltage(),
                rollerMotor.getStatorCurrent(),
                rollerMotor.getDeviceTemp(),
                rollerMotor.getVelocity())
            .isOK();
    inputs.volts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.amps = rollerMotor.getStatorCurrent().getValueAsDouble();
    inputs.celsius = rollerMotor.getDeviceTemp().getValueAsDouble();
    inputs.rotationsPerSecond = rollerMotor.getVelocity().getValueAsDouble();
    inputs.rotations = rollerMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setSurfaceVelocityFeetPerSecond(double targetVelocity) {
    double targetSurfaceVelocityInchPerSecond =
        FeetPerSecond.of(targetVelocity).in(InchesPerSecond);
    double targetRollerVelocityRadiansPerSecond =
        targetSurfaceVelocityInchPerSecond / Constants.ROLLER_RADIUS;
    double targetRotorVelocityRadiansPerSecond =
        targetRollerVelocityRadiansPerSecond / Constants.ROTOR_TO_ROLLER_REDUCTION;
    double targetRotorVelocityRotationsPerSecond =
        RadiansPerSecond.of(targetRotorVelocityRadiansPerSecond).in(RotationsPerSecond);
    if (targetRotorVelocityRotationsPerSecond != 0) {
      rollerMotor.setControl(
          request.withVelocity(-targetRotorVelocityRotationsPerSecond).withSlot(0));
    } else {
      rollerMotor.setControl(request.withVelocity(0).withSlot(1));
    }
  }

  @Override
  public void setVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  @Override
  public void applyNewGains(double[] newGains) {
    /*var config = new TalonFXConfiguration().Slot0;
    config.kP = newGains[0];
    config.kI = newGains[1];
    config.kD = newGains[2];
    config.kS = newGains[3];
    config.kV = newGains[4];
    config.kA = newGains[5];

    rollerMotor.getConfigurator().apply(config);*/
  }
}
