/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;

public class Intake extends SubsystemBase {
    private final RobotState state;
    private final IntakeExtensionIO extensionIO;
    private final IntakeRollerIO rollerIO;
    private final IntakeExtensionIOInputsAutoLogged extensionInputs = new IntakeExtensionIOInputsAutoLogged();
    private final IntakeRollerIOInputsAutoLogged rollerInputs = new IntakeRollerIOInputsAutoLogged();
    private double zeroTimeStamp = Double.NaN;

    private final double ENDSTOP_ZERO_VELOCITY_THRESHOLD_ROTOR_ROTATIONS_PER_SECOND = 0;
    private final double ZERO_VELOCITY_TIME_PERIOD = 0.25;
    private final double ZERO_VOLTAGE = 0.0;
    private final double INTAKE_SURFACE_VELOCITY_FEET_PER_SECOND = 28;

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

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public Intake(IntakeExtensionIO extensionIO, IntakeRollerIO rollerIO, RobotState state) {
        this.extensionIO = extensionIO;
        this.rollerIO = rollerIO;
        this.state = state;
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
    }

    public SystemState handleStateTransitions() {
        switch (wantedState) {
            case INTAKE:
                if (!DriverStation.isDisabled()) {
                    if (Math.abs(extensionInputs.velocity)
                            < ENDSTOP_ZERO_VELOCITY_THRESHOLD_ROTOR_ROTATIONS_PER_SECOND) {
                        if (!Double.isFinite(zeroTimeStamp)) {
                            zeroTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.EXTENDING;
                        } else if ((Timer.getFPGATimestamp() - zeroTimeStamp) >= ZERO_VELOCITY_TIME_PERIOD) {
                            extensionIO.stop();
                            zeroTimeStamp = Double.NaN;
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
                            < ENDSTOP_ZERO_VELOCITY_THRESHOLD_ROTOR_ROTATIONS_PER_SECOND) {
                        if (!Double.isFinite(zeroTimeStamp)) {
                            zeroTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.RETRACTING;
                        } else if ((Timer.getFPGATimestamp() - zeroTimeStamp) >= ZERO_VELOCITY_TIME_PERIOD) {
                            extensionIO.stop();
                            wantedState = WantedState.IDLE;
                            zeroTimeStamp = Double.NaN;
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
    }

    public void handleExtending() {
        extensionIO.setVoltage(ZERO_VOLTAGE);
        rollerIO.setSurfaceVelocityFeetPerSecond(0);
    }

    public void handleRetracting() {
        extensionIO.setVoltage(-ZERO_VOLTAGE);
        rollerIO.setSurfaceVelocityFeetPerSecond(0);
    }

    public void handleIntaking() {
        extensionIO.setVoltage(0);
        rollerIO.setSurfaceVelocityFeetPerSecond(INTAKE_SURFACE_VELOCITY_FEET_PER_SECOND);
    }
}
