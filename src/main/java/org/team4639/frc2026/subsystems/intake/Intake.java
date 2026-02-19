/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
    private final RobotState state;
    private final IntakeExtensionIO extensionIO;
    private final IntakeRollerIO rollerIO;
    private final IntakeExtensionIOInputsAutoLogged extensionInputs = new IntakeExtensionIOInputsAutoLogged();
    private final IntakeRollerIOInputsAutoLogged rollerInputs = new IntakeRollerIOInputsAutoLogged();
    private double zeroTimeStamp = Double.NaN;

    private final double ENDSTOP_ZERO_VELOCITY_THRESHOLD_ROTOR_ROTATIONS_PER_SECOND = 5;
    private final double ENDSTOP_CURRENT_THRESHOLD = 20; // just a guess
    private final double ZERO_VELOCITY_TIME_PERIOD = 0.02;
    private final double ZERO_VOLTAGE_OUT = 6;
    private final double ZERO_VOLTAGE_IN = 3;
    private final double INTAKE_SURFACE_VELOCITY_FEET_PER_SECOND = 28;

    private double retractedRotorPosition = 0.0;
    private double extendedRotorPosition = 1 / Constants.MOTOR_TO_RACK_GEAR_RATIO;

    @Getter
    private final IntakeRollerSysID rollerSysID = new IntakeRollerSysID.IntakeRollerSysIDCTRE(this);

    private IntakeSimulation intakeSimulation;

    public enum WantedState {
        IDLE,
        INTAKE
    }

    public enum SystemState {
        IDLE,
        EXTENDING,
        RETRACTING,
        INTAKE
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public Intake(IntakeExtensionIO extensionIO, IntakeRollerIO rollerIO, RobotState state) {
        this.extensionIO = extensionIO;
        this.rollerIO = rollerIO;
        this.state = state;
        extensionIO.updateInputs(extensionInputs);
        rollerIO.updateInputs(rollerInputs);
        setHomedPositions(extensionInputs.position, Double.NaN);

        setDefaultCommand(run(this::runStateMachine));
    }

    @Override
    public void periodic() {
        extensionIO.updateInputs(extensionInputs);
        rollerIO.updateInputs(rollerInputs);
        Logger.processInputs("Intake Extension", extensionInputs);
        Logger.processInputs("Intake Rollers", rollerInputs);

        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Intake/SystemState", newState.toString());
            systemState = newState;
        }

        state.updateIntakePosition(extensionInputs.position / (extendedRotorPosition - retractedRotorPosition));
        LoggedTunableNumber.ifChanged(hashCode(), nums -> rollerIO.applyNewGains(nums),
                PIDs.rollerkP,
                PIDs.rollerkI,
                PIDs.rollerkD,
                PIDs.rollerkS,
                PIDs.rollerkV,
                PIDs.rollerkA);
    }

    public SystemState handleStateTransitions() {
        switch (wantedState) {
            case INTAKE:
                if (!DriverStation.isDisabled()) {
                    if (Math.abs(extensionInputs.velocity)
                            < ENDSTOP_ZERO_VELOCITY_THRESHOLD_ROTOR_ROTATIONS_PER_SECOND || Math.abs(extensionInputs.current) >= ENDSTOP_CURRENT_THRESHOLD) {
                        if (systemState == SystemState.INTAKE) {
                            return SystemState.INTAKE;
                        } else if (!Double.isFinite(zeroTimeStamp)) {
                            zeroTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.EXTENDING;
                        } else if ((Timer.getFPGATimestamp() - zeroTimeStamp) >= ZERO_VELOCITY_TIME_PERIOD) {
                            extensionIO.stop();
                            zeroTimeStamp = Double.NaN;
                            setHomedPositions(Double.NaN, extensionInputs.position);
                            return SystemState.INTAKE;
                        } else {
                            return SystemState.EXTENDING;
                        }
                    } else {
                        zeroTimeStamp = Double.NaN;
                        return SystemState.EXTENDING;
                    }
                } else {
                    return SystemState.EXTENDING;
                }
            case IDLE:
                if (!DriverStation.isDisabled()) {
                    if (Math.abs(extensionInputs.velocity)
                            < ENDSTOP_ZERO_VELOCITY_THRESHOLD_ROTOR_ROTATIONS_PER_SECOND || Math.abs(extensionInputs.current) >= ENDSTOP_CURRENT_THRESHOLD) {
                        if (systemState == SystemState.IDLE) {
                            return SystemState.IDLE;
                        } else if (!Double.isFinite(zeroTimeStamp)) {
                            zeroTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.RETRACTING;
                        } else if ((Timer.getFPGATimestamp() - zeroTimeStamp) >= ZERO_VELOCITY_TIME_PERIOD) {
                            extensionIO.stop();
                            zeroTimeStamp = Double.NaN;
                            setHomedPositions(Double.NaN, extensionInputs.position);
                            return SystemState.IDLE;
                        } else {
                            return SystemState.RETRACTING;
                        }
                    } else {
                        zeroTimeStamp = Double.NaN;
                        return SystemState.RETRACTING;
                    }
                } else {
                    return SystemState.RETRACTING;
                }
            default:
                return SystemState.IDLE;
        }
    }

    public void handleIdle() {
        extensionIO.stop();
        rollerIO.setSurfaceVelocityFeetPerSecond(0);
        //intakeSimulation.stopIntake();
    }

    public void handleExtending() {
        extensionIO.setVoltage(ZERO_VOLTAGE_OUT);
        rollerIO.setSurfaceVelocityFeetPerSecond(0);
        //intakeSimulation.stopIntake();
    }

    public void handleRetracting() {
        extensionIO.setVoltage(-ZERO_VOLTAGE_IN);
        rollerIO.setSurfaceVelocityFeetPerSecond(0);
       // intakeSimulation.stopIntake();
    }

    public void handleIntaking() {
        extensionIO.setVoltage(0);
        rollerIO.setSurfaceVelocityFeetPerSecond(INTAKE_SURFACE_VELOCITY_FEET_PER_SECOND);
        //intakeSimulation.startIntake();
    }

    public boolean simLaunchFuel() {
        //return intakeSimulation.obtainGamePieceFromIntake();
        return true;
    }

    public void setRollerVoltage(double volts){
        rollerIO.setVoltage(volts);
    }

    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Intake/SystemState", newState.toString());
            systemState = newState;
        }

        switch(systemState){
            case IDLE -> handleIdle();
            case EXTENDING -> handleExtending();
            case RETRACTING -> handleRetracting();
            case INTAKE -> handleIntaking();
        }
    }

    public void setHomedPositions(double retractedRotorPosition, double extendedRotorPosition) {
        if (Double.isNaN(retractedRotorPosition)) {
            this.extendedRotorPosition = extendedRotorPosition;
            this.retractedRotorPosition = extendedRotorPosition - 1 / Constants.MOTOR_TO_RACK_GEAR_RATIO;
        } else {
            this.retractedRotorPosition = retractedRotorPosition;
            this.extendedRotorPosition = retractedRotorPosition +  1 / Constants.MOTOR_TO_RACK_GEAR_RATIO;
        }
    }
}
