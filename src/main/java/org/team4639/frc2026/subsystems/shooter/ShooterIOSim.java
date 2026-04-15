/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.team4639.frc2026.Robot;

public class ShooterIOSim implements ShooterIO {

    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(2), 0.003512, 1), DCMotor.getNeoVortex(2), 0.025);

    private final PIDController flywheelFeedback = new PIDController(0, 0, 0);
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(0, 0);

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;

    public ShooterIOSim() {
        applyNewGains();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        setRPM(flywheelFeedback.getSetpoint());
        flywheelSim.update(Robot.defaultPeriodSecs);

        inputs.leftVolts = leftAppliedVolts;
        inputs.rightVolts = rightAppliedVolts;
        inputs.leftRPM = flywheelSim.getAngularVelocityRPM();
        inputs.rightRPM = -flywheelSim.getAngularVelocityRPM();
    }

    @Override
    public void setVoltage(double appliedVolts) {
        this.leftAppliedVolts = appliedVolts;
        this.rightAppliedVolts = -appliedVolts;
        flywheelSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void setRPM(double targetRPM) {
        flywheelFeedback.setSetpoint(targetRPM);
        leftAppliedVolts = flywheelFeedback.calculate(flywheelSim.getAngularVelocityRPM())
                + flywheelFeedforward.calculate(targetRPM);
        leftAppliedVolts = MathUtil.clamp(leftAppliedVolts, -12, 12);
        rightAppliedVolts = -leftAppliedVolts;
        flywheelSim.setInputVoltage(leftAppliedVolts);
    }

    public void updateGains() {
        flywheelFeedback.setPID(PIDs.shooterKpSim.get(), PIDs.shooterKiSim.get(), PIDs.shooterKdSim.get());
        flywheelFeedforward.setKs(PIDs.shooterKsSim.get());
        flywheelFeedforward.setKv(PIDs.shooterKvSim.get());
        flywheelFeedforward.setKa(PIDs.shooterKaSim.get());
    }

    @Override
    public void applyNewGains() {
        updateGains();
    }
}
