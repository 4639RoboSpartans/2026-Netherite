/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.kicker;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;

public class Kicker extends SubsystemBase {
    private final RobotState state;
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    private final double KICK_RPM = 0;
    private final double IDLE_RPM = 0;

    public enum WantedState {
        IDLE,
        KICK
    }

    public enum SystemState {
        IDLE,
        KICK
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public Kicker(KickerIO io, RobotState state) {
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
            case KICK:
                handleKick();
                break;
        }
    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLE;
            case KICK -> SystemState.KICK;
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
