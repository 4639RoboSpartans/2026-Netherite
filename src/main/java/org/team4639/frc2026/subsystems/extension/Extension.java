/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.subsystems.intake.Constants;
import org.team4639.frc2026.subsystems.intake.Intake;
import org.team4639.lib.util.FullSubsystem;

public class Extension extends FullSubsystem {
    private final RobotState state;
    private final IntakeExtensionIO io;
    private final IntakeExtensionIOInputsAutoLogged inputs;

    private final double ENDSTOP_ZERO_VELOCITY_THRESHOLD_ROTOR_ROTATIONS_PER_SECOND = 5;
    private final double ENDSTOP_CURRENT_THRESHOLD = 20; // just a guess
    private final double ZERO_VELOCITY_TIME_PERIOD = 0.02;
    private final double ZERO_VOLTAGE_OUT = 6;
    private final double ZERO_VOLTAGE_IN = 3;
    private final double ROTOR_SETPOINT_TOLERANCE = 1;

    private final boolean zeroed = false;
    private double zeroTimeStamp = Double.NaN;

    public enum WantedState {
        IDLE,
        EXTENDED,
    }

    public enum SystemState {
        IDLE,
        EXTENDING,
        RETRACTING,
        EXTENDED
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    private double retractedRotorPosition = 0.0;
    private double extendedRotorPosition = Constants.MOTOR_TO_RACK_GEAR_RATIO;

    public Extension(IntakeExtensionIO io, RobotState state) {
        this.io = io;
        this.state = state;
        this.inputs = new IntakeExtensionIOInputsAutoLogged();
        io.updateInputs(inputs);

        setHomedPositions(inputs.position, Double.NaN);
        setDefaultCommand(run(this::runStateMachine));
    }

    public void setHomedPositions(double retractedRotorPosition, double extendedRotorPosition) {
        if (Double.isNaN(retractedRotorPosition)) {
            this.extendedRotorPosition = extendedRotorPosition;
            this.retractedRotorPosition = extendedRotorPosition - Constants.MOTOR_TO_RACK_GEAR_RATIO;
        } else {
            this.retractedRotorPosition = retractedRotorPosition;
            this.extendedRotorPosition = retractedRotorPosition + Constants.MOTOR_TO_RACK_GEAR_RATIO;
        }
    }

    @Override
    public void periodicBeforeScheduler(){
        io.updateInputs(inputs);
        Logger.processInputs("Extension", inputs);

        state.updateIntakePosition(inputs.position / (extendedRotorPosition - retractedRotorPosition));
        state.setExtensionStates(new Pair<>(this.wantedState, this.systemState));
    }

    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Extension/SystemState", newState.toString());
            systemState = newState;
        }

        switch(systemState){
            case IDLE -> handleIdle();
            case EXTENDING -> handleExtending();
            case RETRACTING -> handleRetracting();
            case EXTENDED -> handleExtended();
        }
    }

    public void handleIdle() {
        io.stop();
        io.setBrakeMode(true);
    }

    public void handleExtending() {
        io.setVoltage(ZERO_VOLTAGE_OUT);
        io.setBrakeMode(false);
    }

    public void handleRetracting() {
        io.setVoltage(-ZERO_VOLTAGE_IN);
        io.setBrakeMode(false);
    }

    public void handleExtended() {
        io.setVoltage(0);
        io.setBrakeMode(false);
    }

    public SystemState handleStateTransitions() {
        switch (wantedState) {
            case EXTENDED:
                if (!DriverStation.isDisabled()) {
                    //if intake satisfies zero requirements
                    if (Math.abs(inputs.velocity) < ENDSTOP_ZERO_VELOCITY_THRESHOLD_ROTOR_ROTATIONS_PER_SECOND
                            || Math.abs(inputs.current) >= ENDSTOP_CURRENT_THRESHOLD) {
                        if (systemState == SystemState.EXTENDED) {
                            if (!MathUtil.isNear(inputs.position, extendedRotorPosition, ROTOR_SETPOINT_TOLERANCE)) {
                                return SystemState.EXTENDING;
                            }
                            return SystemState.EXTENDED;
                        } else if (!Double.isFinite(zeroTimeStamp)) {
                            zeroTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.EXTENDING;
                        } else if ((Timer.getFPGATimestamp() - zeroTimeStamp) >= ZERO_VELOCITY_TIME_PERIOD) {
                            io.stop();
                            zeroTimeStamp = Double.NaN;
                            setHomedPositions(Double.NaN, inputs.position);
                            return SystemState.EXTENDED;
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
                    if (Math.abs(inputs.velocity)
                            < ENDSTOP_ZERO_VELOCITY_THRESHOLD_ROTOR_ROTATIONS_PER_SECOND || Math.abs(inputs.current) >= ENDSTOP_CURRENT_THRESHOLD) {
                        if (systemState == SystemState.IDLE) {
                            return SystemState.IDLE;
                        } else if (!Double.isFinite(zeroTimeStamp)) {
                            zeroTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.RETRACTING;
                        } else if ((Timer.getFPGATimestamp() - zeroTimeStamp) >= ZERO_VELOCITY_TIME_PERIOD) {
                            io.stop();
                            zeroTimeStamp = Double.NaN;
                            setHomedPositions(Double.NaN, inputs.position);
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
}
