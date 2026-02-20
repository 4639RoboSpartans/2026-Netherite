/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Voltage;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;
import org.team4639.lib.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

public class Turret extends FullSubsystem {
    private final RobotState state;
    private final TurretIO turretIO;
    private final EncoderIO leftEncoderIO, rightEncoderIO;
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
    private final EncoderIOInputsAutoLogged
            leftEncoderInputs = new EncoderIOInputsAutoLogged(),
            rightEncoderInputs = new EncoderIOInputsAutoLogged();

    private final double IDLE_TURRET_ROTATION = 0;
    private double SCORING_TURRET_ROTATION = 0;
    private double PASSING_TURRET_ROTATION = 0;

    private final double initialTurretRotation;
    private final double initialRotorRotation;

    @Getter
    private final TurretSysID sysID = new TurretSysID.TurretSysIDWPI(this, turretInputs);

    public enum WantedState {
        IDLE,
        SCORING,
        PASSING
    }

    public enum SystemState {
        IDLE,
        SCORING,
        PASSING
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    public Turret(TurretIO turretIO, EncoderIO leftEncoderIO, EncoderIO rightEncoderIO, RobotState state) {
        this.turretIO = turretIO;
        this.leftEncoderIO = leftEncoderIO;
        this.rightEncoderIO = rightEncoderIO;
        this.state = state;
        turretIO.updateInputs(turretInputs);
        leftEncoderIO.updateInputs(leftEncoderInputs);
        rightEncoderIO.updateInputs(rightEncoderInputs);
        initialTurretRotation = getTurretRotation();
        initialRotorRotation = turretInputs.motorPositionRotations;

        setDefaultCommand(this.run(this::runStateMachine));
    }

    @Override
    public void periodicBeforeScheduler() {
        turretIO.updateInputs(turretInputs);
        leftEncoderIO.updateInputs(leftEncoderInputs);
        rightEncoderIO.updateInputs(rightEncoderInputs);
        Logger.processInputs("Turret", turretInputs);
        Logger.processInputs("Left Encoder", leftEncoderInputs);
        Logger.processInputs("Right Encoder", rightEncoderInputs);

        state.updateShooterState(null, null, Rotations.of(getTurretRotationFromRotorRotation()));
    }

    @Override
    public void periodic() {




        if (org.team4639.frc2026.Constants.tuningMode) {
            LoggedTunableNumber.ifChanged(
                hashCode(), turretIO::applyNewGains,
                PIDs.turretKp, PIDs.turretKi, PIDs.turretKd,
                PIDs.turretKs, PIDs.turretKv, PIDs.turretKa,
                PIDs.turretKpSim, PIDs.turretKiSim, PIDs.turretKdSim,
                PIDs.turretKsSim, PIDs.turretKvSim, PIDs.turretKaSim
            );
        }
    }

    private void runStateMachine() {
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Turret/SystemState", newState);
            systemState = newState;
        }

//        if (DriverStation.isDisabled()) {
//            systemState = SystemState.IDLE;
//        }

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

    @Override
    public void periodicAfterScheduler() {
        RobotState.getInstance().setTurretStates(new Pair<>(wantedState, systemState));
        RobotState.getInstance().accept(turretInputs);
    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLE;
            case SCORING -> SystemState.SCORING;
            case PASSING -> SystemState.PASSING;
        };
    }

    // CRT, should only be used on startup to seed position and not while turret is moving
    @AutoLogOutput(key = "TurretRotationsCRT")
    public double getTurretRotation() {
        double leftEncoderRotations = leftEncoderInputs.positionRotations;
        double rightEncoderRotations = rightEncoderInputs.positionRotations;
        double leftMultiple = 1.0 / (Constants.SHARED_GEAR_TEETH / Constants.LEFT_ENCODER_GEAR_TEETH);
        double rightMultiple = 1.0 / (Constants.SHARED_GEAR_TEETH / Constants.RIGHT_ENCODER_GEAR_TEETH);
        double leftDrive = leftEncoderRotations * leftMultiple;
        double rightDrive = rightEncoderRotations * rightMultiple;
        int iterations = 0;
        double closestLeft = leftDrive, closestRight = rightDrive;
        while (!MathUtil.isNear(leftDrive, rightDrive, 0.01) && iterations < 100) {
            if (leftDrive < rightDrive) leftDrive += leftMultiple;
            else rightDrive += rightMultiple;
            iterations++;
            if (Math.abs(leftDrive - rightDrive) < Math.abs(closestLeft - closestRight)) {
                closestLeft = leftDrive;
                closestRight = rightDrive;
            }
        }
        return (closestLeft + closestRight) / Constants.SHARED_GEAR_TO_TURRET_GEAR_RATIO;
    }

    @AutoLogOutput(key = "TurretRotationsCRT2")
    public double getTurretRotation2() {
        double leftEncoderRotations = leftEncoderInputs.positionRotations;
        double rightEncoderRotations = rightEncoderInputs.positionRotations;

        double sharedGearEstimateFromLeftPos = leftEncoderRotations * Constants.LEFT_ENCODER_GEAR_TEETH / Constants.SHARED_GEAR_TEETH;
        double sharedGearEstimateFromRightPos = rightEncoderRotations * Constants.RIGHT_ENCODER_GEAR_TEETH / Constants.SHARED_GEAR_TEETH;

        double sharedGearEstimateFromLeftNeg = leftEncoderRotations * Constants.LEFT_ENCODER_GEAR_TEETH / Constants.SHARED_GEAR_TEETH;
        double sharedGearEstimateFromRightNeg = rightEncoderRotations * Constants.RIGHT_ENCODER_GEAR_TEETH / Constants.SHARED_GEAR_TEETH;

        boolean isPositive = true;
        for(int i = 0; i < 100; i++){
            if (sharedGearEstimateFromLeftPos < sharedGearEstimateFromRightPos){
                sharedGearEstimateFromLeftPos += 1 * Constants.LEFT_ENCODER_GEAR_TEETH / Constants.SHARED_GEAR_TEETH;
            } else {
                sharedGearEstimateFromRightPos += 1 * Constants.RIGHT_ENCODER_GEAR_TEETH / Constants.SHARED_GEAR_TEETH;
            }

            if (Math.abs(sharedGearEstimateFromLeftPos - sharedGearEstimateFromRightPos) < 0.01){
                isPositive = true;
                break;
            }

            if (sharedGearEstimateFromLeftNeg > sharedGearEstimateFromRightNeg){
                sharedGearEstimateFromLeftNeg -= 1 * Constants.LEFT_ENCODER_GEAR_TEETH / Constants.SHARED_GEAR_TEETH;
            } else {
                sharedGearEstimateFromRightNeg -= 1 * Constants.RIGHT_ENCODER_GEAR_TEETH / Constants.SHARED_GEAR_TEETH;
            }

            if (Math.abs(sharedGearEstimateFromLeftNeg - sharedGearEstimateFromRightNeg) < 0.01){
                isPositive = false;
                break;
            }
        }

        return (isPositive
                ? sharedGearEstimateFromLeftPos
                : sharedGearEstimateFromLeftNeg) * (12/92.0);
    }

    @AutoLogOutput(key = "TurretRotations")
    public double getTurretRotationFromRotorRotation() {
        return initialTurretRotation + ((turretInputs.motorPositionRotations - initialRotorRotation) * Constants.MOTOR_TO_TURRET_GEAR_RATIO);
    }

    public double getRotorDeltaRotations(double turretDeltaRotations) {
        return turretDeltaRotations / Constants.MOTOR_TO_TURRET_GEAR_RATIO;
    }

    public double getRotorRotationsFromAbsoluteTurretRotation(double targetTurretRotations) {
        double turretDeltaRotations = targetTurretRotations - initialTurretRotation;
        return initialRotorRotation + getRotorDeltaRotations(turretDeltaRotations);
    }

    public double getNearestTurretRotation(double clampedRotation) {
        if (clampedRotation < 0.4 && clampedRotation > -0.4) {
            return clampedRotation;
        }
        double currentTurretRotation = getTurretRotationFromRotorRotation();
        if (currentTurretRotation * clampedRotation > 0) return clampedRotation;
        if (currentTurretRotation < 0 && clampedRotation >= 0.4) {
            return clampedRotation - 1;
        } else if (currentTurretRotation > 0 && clampedRotation <= -0.4) {
            return clampedRotation + 1;
        } else {
            return clampedRotation;
        }
    }

    private void handleIdle() {
        turretIO.setRotorRotationSetpoint(getRotorRotationsFromAbsoluteTurretRotation(IDLE_TURRET_ROTATION));
    }

    private void handleScoring() {
        double nearestTurretRotation = getNearestTurretRotation(SCORING_TURRET_ROTATION);
        turretIO.setRotorRotationSetpoint(getRotorRotationsFromAbsoluteTurretRotation(nearestTurretRotation));
    }

    private void handlePassing() {
        double nearestTurretRotation = getNearestTurretRotation(PASSING_TURRET_ROTATION);
        turretIO.setRotorRotationSetpoint(getRotorRotationsFromAbsoluteTurretRotation(nearestTurretRotation));
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(WantedState wantedState, double SCORING_TURRET_ROTATION, double PASSING_TURRET_ROTATION) {
        this.SCORING_TURRET_ROTATION = SCORING_TURRET_ROTATION;
        this.PASSING_TURRET_ROTATION = PASSING_TURRET_ROTATION;
        setWantedState(wantedState);
    }

    public double getTurretSetpoint() {
        return switch (systemState) {
            case IDLE -> IDLE_TURRET_ROTATION;
            case SCORING -> SCORING_TURRET_ROTATION;
            case PASSING -> PASSING_TURRET_ROTATION;
        };
    }

    public double getRotorSetpoint() {
        return getRotorRotationsFromAbsoluteTurretRotation(getTurretSetpoint());
    }

    public boolean atSetpoint() {
        return MathUtil.isNear(getRotorSetpoint(), turretInputs.motorPositionRotations, Constants.ROTOR_ROTATION_TOLERANCE)
                || MathUtil.isNear(getRotorSetpoint() + 1.0 / Constants.MOTOR_TO_TURRET_GEAR_RATIO, turretInputs.motorPositionRotations, Constants.ROTOR_ROTATION_TOLERANCE)
                || MathUtil.isNear(getRotorSetpoint() - 1.0 / Constants.MOTOR_TO_TURRET_GEAR_RATIO, turretInputs.motorPositionRotations, Constants.ROTOR_ROTATION_TOLERANCE);
    }

    protected void setVoltage(Voltage volts){
        turretIO.setVoltage(volts.in(Volts));
    }
}
