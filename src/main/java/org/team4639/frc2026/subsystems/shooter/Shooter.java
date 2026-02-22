/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.Constants;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;
import org.team4639.lib.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;

public class Shooter extends FullSubsystem {
    private final RobotState state;
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final double PASSING_RPM = 0;
    private final double IDLE_VOLTAGE = 0;
    @AutoLogOutput(key = "Shooter Scoring RPM")
    private double SCORING_RPM = 0;

    private final double SHOOTING_RPM_TOLERANCE = 50;

    @Getter
    private final ShooterSysID sysID = new ShooterSysID.ShooterSysIDWPI(this, inputs);

    public enum WantedState {
        OFF,
        IDLE,
        SCORING,
        PASSING
    }

    public enum SystemState {
        OFF,
        IDLE,
        SCORING,
        PASSING
    }

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;

    public Shooter(ShooterIO io, RobotState state) {
        this.io = io;
        this.state = state;
    }

    @Override
    public void periodicBeforeScheduler() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        state.updateShooterState(Rotations.per(Minute).of(inputs.leftRPM), null, null);

        this.setDefaultCommand(this.run(this::runStateMachine));
    }

    @Override
    public void periodic() {

        if (Constants.tuningMode) {
            LoggedTunableNumber.ifChanged(
                hashCode(), io::applyNewGains,
                PIDs.shooterKp, PIDs.shooterKi, PIDs.shooterKd,
                PIDs.shooterKs, PIDs.shooterKv, PIDs.shooterKa,
                PIDs.shooterKpSim, PIDs.shooterKiSim, PIDs.shooterKdSim,
                PIDs.shooterKsSim, PIDs.shooterKvSim, PIDs.shooterKaSim
            );
        }
    }

    @Override
    public void periodicAfterScheduler() {
        RobotState.getInstance().setShooterStates(new Pair<>(wantedState, systemState));
        RobotState.getInstance().accept(inputs);
    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case SCORING -> SystemState.SCORING;
            case PASSING -> SystemState.PASSING;
            case IDLE -> SystemState.IDLE;
            default -> SystemState.OFF;
        };
    }

    private void handleOff() {
        io.setVoltage(0);
    }

    private void handleScoring() {
        io.setRPM(this.SCORING_RPM);
    }

    private void handlePassing() {
        io.setRPM(PASSING_RPM);
    }

    private void handleIdle() {
        io.setVoltage(IDLE_VOLTAGE);
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(WantedState wantedState, double scoringRPM) {
        setWantedState(wantedState);
        this.SCORING_RPM = scoringRPM;
    }

    /**
     * Should not be called in comp code. All usages of
     * setVoltage() needed for comp should be called internally.
     * @param volts voltage to set shooter motors to
     */
    protected void setVoltage(Voltage volts) {
        io.setVoltage(volts.in(Volts));
    }

    public double getSetpointRPM() {
        return switch (systemState) {
            case SCORING -> SCORING_RPM;
            case PASSING -> PASSING_RPM;
            default -> 0;
        };
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(getSetpointRPM(), -inputs.leftRPM, SHOOTING_RPM_TOLERANCE);
    }

    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Shooter/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.OFF;
        }

        switch (systemState) {
            case OFF:
                handleOff();
                break;
            case IDLE:
                handleIdle();
                break;
            case SCORING:
                handleScoring();
                break;
            case PASSING:
                handlePassing();
                break;
        }
    }
}
