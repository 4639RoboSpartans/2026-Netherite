/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.turret;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.team4639.frc2026.Robot;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim turretSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.0000001, 1),
          DCMotor.getKrakenX44(1));
  private final ProfiledPIDController turretPIDController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(120, 200));
  private double appliedVolts = 0.0;
  private double goalRotation = 0.0;

  public TurretIOSim() {
    applyNewGains();
  }

  @Override
  public void updateInputs(TurretIO.TurretIOInputs inputs) {
    setRotorRotationSetpoint(goalRotation);
    turretSim.update(Robot.defaultPeriodSecs);

    inputs.motorVoltage = appliedVolts;
    inputs.motorPositionRotations = turretSim.getAngularPositionRotations();
    inputs.motorVelocity = turretSim.getAngularVelocity().in(RotationsPerSecond);

    inputs.motorPositionsRotations = new double[] {turretSim.getAngularPositionRotations()};
    inputs.motorPositionsTimestamps = new double[] {Timer.getFPGATimestamp()};
  }

  @Override
  public void setRotorRotationSetpoint(double rotation) {
    goalRotation = rotation;
    turretPIDController.setGoal(rotation);
    appliedVolts = turretPIDController.calculate(turretSim.getAngularPositionRotations());
    appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);
    turretSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setRotorRotationSetpoint(double rotation, double velocityrps) {
    setRotorRotationSetpoint(rotation);
  }

  public void updateGains() {
    turretPIDController.setPID(
        PIDs.turretKpSim.get(), PIDs.turretKiSim.get(), PIDs.turretKdSim.get());
  }

  @Override
  public void applyNewGains() {
    updateGains();
  }
}
