/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.extension;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.team4639.frc2026.Robot;
import org.team4639.frc2026.subsystems.intake.Constants;

public class IntakeExtensionIOSim implements IntakeExtensionIO {
  private final SingleJointedArmSim extensionSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1),
          Constants.MOTOR_TO_RACK_GEAR_RATIO,
          0.0001,
          0.5,
          0,
          Units.rotationsToRadians(1),
          false,
          0,
          0.001,
          0.0001);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeExtensionIOInputs inputs) {
    setVoltage(appliedVolts);
    extensionSim.update(Robot.defaultPeriodSecs);

    inputs.volts = appliedVolts;
    inputs.amps = extensionSim.getCurrentDrawAmps();
    inputs.rotationsPerSecond =
        Units.radiansToRotations(extensionSim.getVelocityRadPerSec())
            / Constants.MOTOR_TO_RACK_GEAR_RATIO;
    inputs.rotations =
        Units.radiansToRotations(extensionSim.getAngleRads()) / Constants.MOTOR_TO_RACK_GEAR_RATIO;
  }

  @Override
  public void setVoltage(double appliedVoltage) {
    appliedVolts = appliedVoltage;
    extensionSim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    extensionSim.setInputVoltage(0);
  }
}
