/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import static org.team4639.frc2026.subsystems.hood.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.team4639.frc2026.Robot;

public class HoodIOSim implements HoodIO {
  private final SingleJointedArmSim hoodSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1),
          MOTOR_TO_HOOD_GEAR_RATIO,
          0.0000000001,
          Units.inchesToMeters(7.156),
          Units.rotationsToRadians(HOOD_ENCODER_MIN_ROTATION),
          Units.rotationsToRadians(HOOD_ENCODER_MAX_ROTATION),
          false,
          Units.rotationsToRadians(HOOD_ENCODER_MIN_ROTATION),
          0.00001,
          0.0001);
  private final ProfiledPIDController hoodPIDController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(10, 10));
  private double appliedVolts = 0.0;
  private double pivotSetpointDegrees = 0.0;

  public HoodIOSim() {
    applyNewGains();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    setSetpointDegrees(pivotSetpointDegrees);
    hoodSim.update(Robot.defaultPeriodSecs);

    inputs.pivotVoltage = appliedVolts;
    inputs.pivotPositionDegrees =
        (Units.radiansToRotations(hoodSim.getAngleRads()) - HOOD_ENCODER_MIN_ROTATION)
                / ENCODER_ROTATIONS_PER_DEGREE
            + HOOD_MIN_ANGLE_DEGREES;
    inputs.pivotVelocityDegrees = Units.radiansToDegrees(hoodSim.getVelocityRadPerSec());
  }

  @Override
  public void setSetpointDegrees(double setpointDegrees) {
    pivotSetpointDegrees = setpointDegrees;
    double rotation =
        (setpointDegrees - HOOD_MIN_ANGLE_DEGREES) * ENCODER_ROTATIONS_PER_DEGREE
            + HOOD_ENCODER_MIN_ROTATION;
    rotation = MathUtil.clamp(rotation, HOOD_ENCODER_MIN_ROTATION, HOOD_ENCODER_MAX_ROTATION);
    hoodPIDController.setGoal(rotation);
    appliedVolts = hoodPIDController.calculate(Units.radiansToRotations(hoodSim.getAngleRads()));
    appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);
    hoodSim.setInputVoltage(appliedVolts);
  }

  public void updateGains() {
    hoodPIDController.setPID(PIDs.hoodKpSim.get(), PIDs.hoodKiSim.get(), PIDs.hoodKdSim.get());
  }

  @Override
  public void setPosition(double positionDegrees) {
    hoodSim.setState(Units.degreesToRadians(positionDegrees), 0);
  }

  @Override
  public void applyNewGains() {
    updateGains();
  }
}
