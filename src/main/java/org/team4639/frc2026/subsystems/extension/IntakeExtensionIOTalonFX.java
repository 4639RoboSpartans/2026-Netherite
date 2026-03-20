/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.extension;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class IntakeExtensionIOTalonFX implements IntakeExtensionIO {
  private final TalonFX extensionMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VoltageOut request = new VoltageOut(0);

  public IntakeExtensionIOTalonFX(PortConfiguration ports) {
    extensionMotor = Phoenix6Factory.createDefaultTalon(ports.IntakeExtensionMotorID, false);

    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 15;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(config));

    RobotState.disabled.onTrue(
        Commands.runOnce(() -> extensionMotor.setNeutralMode(NeutralModeValue.Coast))
            .ignoringDisable(true));
    RobotState.disabled.onFalse(
        Commands.runOnce(() -> extensionMotor.setNeutralMode(NeutralModeValue.Brake))
            .ignoringDisable(true));
  }

  @Override
  public void updateInputs(IntakeExtensionIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                extensionMotor.getMotorVoltage(),
                extensionMotor.getStatorCurrent(),
                extensionMotor.getDeviceTemp(),
                extensionMotor.getVelocity())
            .isOK();
    inputs.volts = extensionMotor.getMotorVoltage().getValueAsDouble();
    inputs.amps = extensionMotor.getStatorCurrent().getValueAsDouble();
    inputs.celsius = extensionMotor.getDeviceTemp().getValueAsDouble();
    inputs.rotationsPerSecond = extensionMotor.getVelocity().getValueAsDouble();
    inputs.rotations = extensionMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setVoltage(double appliedVoltage) {
    extensionMotor.setControl(request.withOutput(appliedVoltage));
  }

  @Override
  public void stop() {
    extensionMotor.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean brake) {
    extensionMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
