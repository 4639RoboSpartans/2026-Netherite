/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.spindexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;

public class Spindexer extends SubsystemBase {
    private final RobotState state;
    private final SpindexerIO io;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    private final double KICK_RPM = 0;
    private final double IDLE_RPM = 0;

    public enum WantedState {
        IDLE,
        SPIN
    }

    public enum SystemState {
        IDLE,
        SPIN
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public Spindexer(SpindexerIO io, RobotState state) {
        this.io = io;
        this.state = state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Kicker", inputs);
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Kicker/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLE;
        }

        switch (systemState) {
            case IDLE:
                handleIdle();
                break;
            case SPIN:
                handleKick();
                break;
        }
    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLE;
            case SPIN -> SystemState.SPIN;
        };
    }

    private void handleIdle() {
        io.setRotorVelocityRPM(IDLE_RPM);
    }

    private void handleKick() {
        io.setRotorVelocityRPM(KICK_RPM);
    }

    private void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }
}
