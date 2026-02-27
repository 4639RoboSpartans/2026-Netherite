/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;
import org.team4639.lib.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;

public class Hood extends FullSubsystem {
    private final RobotState state;
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private final double PASSING_HOOD_ANGLE = 0;
    private final double IDLE_HOOD_ANGLE = 0;

    @AutoLogOutput(key = "Hood Setpoint Degrees")
    private double SCORING_HOOD_ANGLE = 0;

    private final double HOOD_TOLERANCE_DEGREES = 5;

    @Getter
    private final HoodSysID sysID = new HoodSysID.HoodSysIDWPI(this, inputs);

    public enum WantedState {
        IDLE,
        SCORING,
        PASSING
    }

    public enum SystemState {
        HOME,
        IDLE,
        SCORING,
        PASSING
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.HOME;

    public Hood(HoodIO io, RobotState state) {
        this.io = io;
        this.state = state;

        this.setDefaultCommand(this.run(this::runStateMachine));
    }

    @Override
    public void periodicBeforeScheduler() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
        state.updateShooterState(null, Degrees.of(inputs.pivotPositionDegrees), null);
    }

    @Override
    public void periodic() {

        if (org.team4639.frc2026.Constants.tuningMode) {
            LoggedTunableNumber.ifChanged(
                hashCode(), io::applyNewGains,
                PIDs.hoodKp, PIDs.hoodKi, PIDs.hoodKd,
                PIDs.hoodKs, PIDs.hoodKv, PIDs.hoodKa,
                PIDs.hoodKpSim, PIDs.hoodKiSim, PIDs.hoodKdSim
            );
        }
    }

    @Override
    public void periodicAfterScheduler() {
        //TODO: use PIV state
        RobotState.getInstance().setHoodStates(new Pair<>(wantedState, systemState));
        RobotState.getInstance().accept(inputs);
    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case IDLE -> {
                if (systemState == SystemState.HOME){
                    if (Math.abs(inputs.pivotCurrent) > 20){
                        io.setPosition(Constants.HOOD_MIN_ANGLE_DEGREES);
                        yield SystemState.IDLE;
                    } else {
                        yield SystemState.HOME;
                    }
                } else {
                    yield SystemState.IDLE;
                }
            }
            case SCORING -> SystemState.SCORING;
            case PASSING -> SystemState.PASSING;
        };
    }

    private void handleHome() {
        io.setVoltage(-3);
    }

    private void handleIdle() {
        io.setSetpointDegrees(IDLE_HOOD_ANGLE);
    }

    private void handleScoring() {
        io.setSetpointDegrees(SCORING_HOOD_ANGLE);
    }

    private void handlePassing() {
        io.setSetpointDegrees(PASSING_HOOD_ANGLE);
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(WantedState wantedState, double scoringAngleRotations) {
        setWantedState(wantedState);
        this.SCORING_HOOD_ANGLE = Rotations.of(scoringAngleRotations).in(Degrees);
    }

    /**
     * Should not be called in comp code. All usages of
     * setVoltage() needed for comp should be called internally.
     * @param volts
     */
    public void setVoltage(Voltage volts){
        io.setVoltage(volts.in(Volts));
    }

    public double getSetpointAngle() {
        return switch (systemState) {
            case SCORING -> SCORING_HOOD_ANGLE;
            case PASSING -> PASSING_HOOD_ANGLE;
            default -> Constants.HOOD_MIN_ANGLE_DEGREES;
        };
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(getSetpointAngle(), inputs.pivotPositionDegrees, HOOD_TOLERANCE_DEGREES);
    }

    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Hood/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLE;
        }

        switch (systemState) {
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
