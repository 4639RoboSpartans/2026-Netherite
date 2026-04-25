/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;

public class Extension extends FullSubsystem {

    private final RobotState state;
    private final IntakeExtensionIO io;
    private final IntakeExtensionIOInputsAutoLogged inputs;

    // any RPS greater than this is counted as not moving backwards. Used
    // for calculating when the intake is no longer being pushed after a
    // collision.
    public static final double INTAKE_NOT_MOVING_BACKWARD_RPS = -0.1;
    // rotor RPS tolerance for zeroing the intake against either of its hardstops.
    private final double ENDSTOP_ZERO_VELOCITY_THRESHOLD_ROTOR_ROTATIONS_PER_SECOND = 5;
    // current thresholds at which we determine the motor has reached its hardstops.
    private final double ENDSTOP_CURRENT_THRESHOLD_OUT = 19;
    private final double ENDSTOP_CURRENT_THRESHOLD_IN = 19;
    // minimum elapsed time for zeroing. Prevents initial current spikes from
    // causing false positives.
    private final double ZERO_VELOCITY_TIME_PERIOD = 0.02;
    private final double ZERO_VOLTAGE_OUT = 4;
    private final double REZERO_VOLTAGE_OUT = 2;
    private final double ZERO_VOLTAGE_IN = 3;
    private final double ROTOR_SETPOINT_TOLERANCE = 5;

    // maximum elapsed time for zeroing. Prevents intake from being unusable
    // in case it never reaches its current threshold (eg; if the rack shears)
    private final double ZERO_TIMEOUT = 1.5;

    @Setter
    private double MANUAL_VOLTAGE = 0;

    // whether a rezero is needed (collision detection)
    private boolean rezero = false;

    private double zeroTimeStamp = Double.NaN;

    public enum WantedState {
        IDLE,
        EXTENDED,
        MANUAL_VOLTAGE
    }

    public enum SystemState {
        IDLE,
        EXTENDING,
        RETRACTING,
        EXTENDED,
        MANUAL_VOLTAGE
    }

    @Setter
    private WantedState wantedState = WantedState.IDLE;

    private SystemState systemState = SystemState.IDLE;

    // initial estimates for rotor positions. These are updated when the
    // intake is zeroed against its hardstops.
    private double retractedRotorPosition = 0.0;
    private double extendedRotorPosition = Constants.MOTOR_TO_RACK_GEAR_RATIO;

    public final Subsystem dummy = new Subsystem() {};

    public Extension(IntakeExtensionIO io, RobotState state) {
        this.io = io;
        this.state = state;
        this.inputs = new IntakeExtensionIOInputsAutoLogged();
        io.updateInputs(inputs);

        setHomedPositions(inputs.rotations, Double.NaN);
        setDefaultCommand(run(this::runStateMachine));

        Logger.recordOutput("Extension/SystemState", systemState.toString());
    }

    /**
     * Updates the idle and extended positions of the motor after a rezero. Uses a
     * Double.NaN flag to determine which position is known and which should be
     * calculated using the measured range of the motor.
     */
    public void setHomedPositions(double retractedRotorPosition, double extendedRotorPosition) {
        if (Double.isNaN(retractedRotorPosition)) {
            this.extendedRotorPosition = extendedRotorPosition;
            this.retractedRotorPosition =
                    extendedRotorPosition - org.team4639.frc2026.subsystems.extension.Constants.ROTOR_RANGE;
        } else {
            this.retractedRotorPosition = retractedRotorPosition;
            this.extendedRotorPosition =
                    retractedRotorPosition + org.team4639.frc2026.subsystems.extension.Constants.ROTOR_RANGE;
        }
    }

    @Override
    public void periodicBeforeScheduler() {
        io.updateInputs(inputs);
        Logger.processInputs("Extension", inputs);

        state.updateIntakePosition(
                (inputs.rotations - retractedRotorPosition) / (extendedRotorPosition - retractedRotorPosition));
    }

    @Override
    public void periodicAfterScheduler() {
        state.setExtensionStates(new Pair<>(this.wantedState, this.systemState));

        state.acceptCANMeasurement(inputs.connected);
        state.acceptTemperatureMeasurement(inputs.celsius);
    }

    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Extension/SystemState", newState.toString());
            systemState = newState;
        }

        switch (systemState) {
            case IDLE -> handleIdle();
            case EXTENDING -> handleExtending();
            case RETRACTING -> handleRetracting();
            case EXTENDED -> handleExtended();
            case MANUAL_VOLTAGE -> handleManualVoltage();
        }
    }

    public void handleIdle() {
        io.stop();
        io.setBrakeMode(true);
    }

    public void handleExtending() {
        io.setVoltage(rezero ? REZERO_VOLTAGE_OUT : ZERO_VOLTAGE_OUT);
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

    public void handleManualVoltage() {
        io.setVoltage(MANUAL_VOLTAGE);
    }

    public SystemState handleStateTransitions() {
        switch (wantedState) {
            case EXTENDED:
                if (!DriverStation.isDisabled()) {
                    // if intake satisfies zero requirements
                    if (Math.abs(inputs.rotationsPerSecond) < ENDSTOP_ZERO_VELOCITY_THRESHOLD_ROTOR_ROTATIONS_PER_SECOND
                            || Math.abs(inputs.amps) >= ENDSTOP_CURRENT_THRESHOLD_OUT
                            || (Timer.getFPGATimestamp() - zeroTimeStamp) > ZERO_TIMEOUT) {
                        if (systemState == SystemState.EXTENDED) {
                            if (!MathUtil.isNear(inputs.rotations, extendedRotorPosition, ROTOR_SETPOINT_TOLERANCE)) {
                                if (inputs.rotationsPerSecond > INTAKE_NOT_MOVING_BACKWARD_RPS) {
                                    rezero = true;
                                    return SystemState.EXTENDING;
                                } else {
                                    return SystemState.EXTENDED;
                                }
                            }
                            return SystemState.EXTENDED;
                        } else if (!Double.isFinite(zeroTimeStamp)) {
                            zeroTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.EXTENDING;
                        } else if ((Timer.getFPGATimestamp() - zeroTimeStamp) >= ZERO_VELOCITY_TIME_PERIOD) {
                            io.stop();
                            zeroTimeStamp = Double.NaN;
                            setHomedPositions(Double.NaN, inputs.rotations);
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
                rezero = false;
                if (!DriverStation.isDisabled()) {
                    if (Math.abs(inputs.rotationsPerSecond) < ENDSTOP_ZERO_VELOCITY_THRESHOLD_ROTOR_ROTATIONS_PER_SECOND
                            || Math.abs(inputs.amps) >= ENDSTOP_CURRENT_THRESHOLD_IN
                            || (Timer.getFPGATimestamp() - zeroTimeStamp) > ZERO_TIMEOUT) {
                        if (systemState == SystemState.IDLE) {
                            return SystemState.IDLE;
                        } else if (!Double.isFinite(zeroTimeStamp)) {
                            zeroTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.RETRACTING;
                        } else if ((Timer.getFPGATimestamp() - zeroTimeStamp) >= ZERO_VELOCITY_TIME_PERIOD) {
                            io.stop();
                            zeroTimeStamp = Double.NaN;
                            setHomedPositions(inputs.rotations, Double.NaN);
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
            case MANUAL_VOLTAGE:
                return SystemState.MANUAL_VOLTAGE;
            default:
                return SystemState.IDLE;
        }
    }
}
