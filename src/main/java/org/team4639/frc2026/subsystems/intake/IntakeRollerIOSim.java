/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.team4639.frc2026.Robot;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private final FlywheelSim rollerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX44(1), 0.000001, Constants.ROTOR_TO_ROLLER_REDUCTION),
          DCMotor.getKrakenX44(1),
          0.001);
  private final PIDController rollerFeedback = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward rollerFeedforward = new SimpleMotorFeedforward(0, 0.35);
  private double appliedVolts = 0.0;
  private double targetVelocity = 0.0;

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    setSurfaceVelocityFeetPerSecond(targetVelocity);
    rollerSim.update(Robot.defaultPeriodSecs);

    inputs.voltage = appliedVolts;
    inputs.velocity =
        Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec())
            / Constants.ROTOR_TO_ROLLER_REDUCTION;
  }

  @Override
  public void setSurfaceVelocityFeetPerSecond(double targetVelocity) {
    this.targetVelocity = targetVelocity;
    double targetSurfaceVelocityInchPerSecond =
        FeetPerSecond.of(targetVelocity).in(InchesPerSecond);
    double targetRollerVelocityRadiansPerSecond =
        targetSurfaceVelocityInchPerSecond / Constants.ROLLER_RADIUS;
    double targetRotorVelocityRadiansPerSecond =
        targetRollerVelocityRadiansPerSecond / Constants.ROTOR_TO_ROLLER_REDUCTION;
    double targetRotorVelocityRotationsPerSecond =
        RadiansPerSecond.of(targetRotorVelocityRadiansPerSecond).in(RotationsPerSecond);
    rollerFeedback.setSetpoint(targetRotorVelocityRotationsPerSecond);
    appliedVolts =
        rollerFeedback.calculate(
                Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec())
                    / Constants.ROTOR_TO_ROLLER_REDUCTION)
            + rollerFeedforward.calculate(targetRotorVelocityRotationsPerSecond);
    rollerSim.setInputVoltage(appliedVolts);
  }
}
