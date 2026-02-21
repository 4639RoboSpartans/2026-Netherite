/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intakeRollers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.LoggedTunableNumber;

public class IntakeRollers extends SubsystemBase {
    private final RobotState state;
    private final IntakeRollerIO io;
    private final IntakeRollerIOInputsAutoLogged rollerInputs = new IntakeRollerIOInputsAutoLogged();

    private final double INTAKE_SURFACE_VELOCITY_FEET_PER_SECOND = 28;

    @Getter
    private final IntakeRollerSysID rollerSysID = new IntakeRollerSysID.IntakeRollerSysIDCTRE(this);

    public enum WantedState {
        IDLE,
        INTAKE
    }

    public enum SystemState {
        IDLE,
        INTAKE
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public IntakeRollers(IntakeRollerIO io, RobotState state) {
        this.io = io;
        this.state = state;
        io.updateInputs(rollerInputs);

        setDefaultCommand(run(this::runStateMachine));
    }

    @Override
    public void periodic() {
        io.updateInputs(rollerInputs);
        Logger.processInputs("Intake Rollers", rollerInputs);

        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("IntakeRollers/SystemState", newState.toString());
            systemState = newState;
        }

        LoggedTunableNumber.ifChanged(hashCode(), nums -> io.applyNewGains(nums),
                PIDs.rollerkP,
                PIDs.rollerkI,
                PIDs.rollerkD,
                PIDs.rollerkS,
                PIDs.rollerkV,
                PIDs.rollerkA);
    }

    public SystemState handleStateTransitions() {
        return switch (wantedState) {
            case INTAKE -> SystemState.INTAKE;
            case IDLE -> SystemState.IDLE;
        };
    }

    public void handleIdle() {
        io.setSurfaceVelocityFeetPerSecond(0);
    }

    public void handleIntaking() {
        io.setSurfaceVelocityFeetPerSecond(INTAKE_SURFACE_VELOCITY_FEET_PER_SECOND);
    }

    public void setRollerVoltage(double volts){
        io.setVoltage(volts);
    }

    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Intake/SystemState", newState.toString());
            systemState = newState;
        }

        switch(systemState){
            case IDLE -> handleIdle();
            case INTAKE -> handleIntaking();
        }
    }
}
