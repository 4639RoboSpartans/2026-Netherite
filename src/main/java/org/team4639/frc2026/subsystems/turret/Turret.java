/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;
import org.team4639.lib.util.LoggedTunableNumber;

import java.nio.Buffer;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.locks.ReentrantLock;

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

    private final TurretSetpoint IDLE_TURRET_ROTATION = new TurretSetpoint(0, 0);
    private TurretSetpoint SCORING_TURRET_ROTATION = new TurretSetpoint(0, 0);
    private TurretSetpoint PASSING_TURRET_ROTATION = new TurretSetpoint(0, 0);
    private TurretSetpoint HUB_TRACK_TURRET_ROTATION = new TurretSetpoint(0, 0);

    private double initialTurretRotation;
    private double initialRotorRotation;

    protected static final double ODOMETRY_FREQUENCY = CANBus.roboRIO().isNetworkFD() ? 250.0 : 100.0;

    protected static final ReentrantLock motorPositionLock = new ReentrantLock();

    @Getter
    private final TurretSysID sysID = new TurretSysID.TurretSysIDWPI(this, turretInputs);

    private final Debouncer turretRezeroDebouncer = new Debouncer(0.5);
    private final Queue<Double> CRTMeasurements = new LinkedList<>();

    public enum WantedState {
        IDLE,
        SCORING,
        PASSING,
        HUB_TRACK
    }

    public enum SystemState {
        IDLE,
        SCORING,
        PASSING,
        HUB_TRACK
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
        initialTurretRotation = getTurretRotation(leftEncoderInputs.positionRotations, rightEncoderInputs.positionRotations);
        initialRotorRotation = turretInputs.motorPositionRotations;

        setDefaultCommand(this.run(this::runStateMachine));
        Logger.recordOutput("Turret/SystemState", systemState);
        PhoenixOdometryThread.getInstance().start();
    }

    @Override
    public void periodicBeforeScheduler() {
        motorPositionLock.lock();
        turretIO.updateInputs(turretInputs);
        leftEncoderIO.updateInputs(leftEncoderInputs);
        rightEncoderIO.updateInputs(rightEncoderInputs);
        Logger.processInputs("Turret", turretInputs);
        Logger.processInputs("Left Encoder", leftEncoderInputs);
        Logger.processInputs("Right Encoder", rightEncoderInputs);
        motorPositionLock.unlock();

        state.updateShooterState(null, null, Rotations.of(getTurretRotationFromRotorRotation()));

        double[] turretRotations = Arrays.stream(turretInputs.motorPositionsRotations).map(this::getTurretRotationFromRotorRotation).toArray();
        double[] timestamps = turretInputs.motorPositionsTimestamps;

        for (int i = 0; i < timestamps.length; i ++){
            state.acceptTurretMeasurement(turretRotations[i], timestamps[i]);
        }
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

        var speeds = state.getChassisSpeeds();
        if (MathUtil.isNear(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), 0, 1E-3)
                && MathUtil.isNear(speeds.omegaRadiansPerSecond, 0, 1E-3)
                && MathUtil.isNear(turretInputs.motorVelocity, 0, 1E-3)) {
            CRTMeasurements.add(CRT());
            if (turretRezeroDebouncer.calculate(true)) {
                var averageCRT = CRTMeasurements.stream().reduce(Double::sum).orElse(-1.0) / CRTMeasurements.size();

                boolean allInTolerance = true;
                var isCRTStable = CRTMeasurements.stream().anyMatch(measurement -> !MathUtil.isNear(measurement, averageCRT, 1E-2));

                if (isCRTStable && Constants.TURRET_MIN_ROTATIONS <= averageCRT && averageCRT <= Constants.TURRET_MAX_ROTATIONS) {
                    initialTurretRotation = getTurretRotation(leftEncoderInputs.positionRotations, rightEncoderInputs.positionRotations);
                    initialRotorRotation = turretInputs.motorPositionRotations;

                    CRTMeasurements.clear();
                }
            }
        } else {
            turretRezeroDebouncer.calculate(false);
            CRTMeasurements.clear();
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
            case HUB_TRACK:
                handleHubTrack();
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
            case HUB_TRACK -> SystemState.HUB_TRACK;
        };
    }

    // CRT, should only be used on startup to seed position and not while turret is moving
    public double getTurretRotation(double leftEncoderRotations, double rightEncoderRotations) {
        double abs1 =
                MathUtil.inputModulus(
                        leftEncoderRotations,
                        0.0,
                        1.0) % 1;
        double abs2 =
                MathUtil.inputModulus(
                        rightEncoderRotations,
                        0.0,
                        1.0) % 1;
        double ratio1 = Constants.SHARED_GEAR_TO_TURRET_GEAR_RATIO * Constants.SHARED_GEAR_TEETH / Constants.LEFT_ENCODER_GEAR_TEETH;
        double ratio2 = Constants.SHARED_GEAR_TO_TURRET_GEAR_RATIO * Constants.SHARED_GEAR_TEETH / Constants.RIGHT_ENCODER_GEAR_TEETH;
        double bestErr = Double.MAX_VALUE;
        double bestRot = Double.NaN;

        double nMinD = Math.min(ratio1 * Constants.TURRET_EXTENDED_MIN_ROTATIONS, ratio1 * Constants.TURRET_EXTENDED_MAX_ROTATIONS) - abs1;
        double nMaxD = Math.max(ratio1 * Constants.TURRET_EXTENDED_MIN_ROTATIONS, ratio1 * Constants.TURRET_EXTENDED_MAX_ROTATIONS) - abs1;
        int minN = (int) Math.floor(nMinD) - 1;
        int maxN = (int) Math.ceil(nMaxD) + 1;

        for (int n = minN; n <= maxN; n++) {
            double mechRot = (abs1 + n) / ratio1;
            if (mechRot < Constants.TURRET_EXTENDED_MIN_ROTATIONS - 1e-6 || mechRot > Constants.TURRET_EXTENDED_MAX_ROTATIONS + 1e-6) {
                continue;
            }

            double predicted2 = MathUtil.inputModulus(ratio2 * mechRot, 0.0, 1.0);
            double err = modularError(predicted2, abs2);

            if (err < bestErr) {
                bestErr = err;
                bestRot = mechRot;
            }
        }
        return bestRot;
    }

    private static double modularError(double a, double b) {
        double diff = Math.abs(a - b);
        return diff > 0.5 ? 1.0 - diff : diff;
    }

    @AutoLogOutput(key = "TurretRotations")
    public double getTurretRotationFromRotorRotation() {
        return getTurretRotationFromRotorRotation(turretInputs.motorPositionRotations);
    }

    private double getTurretRotationFromRotorRotation(double rotorRotation){
        return initialTurretRotation + ((rotorRotation - initialRotorRotation) * Constants.MOTOR_TO_TURRET_GEAR_RATIO);
    }

    public double getRotorDeltaRotations(double turretDeltaRotations) {
        return turretDeltaRotations / Constants.MOTOR_TO_TURRET_GEAR_RATIO;
    }

    public double getRotorRotationsFromAbsoluteTurretRotation(double targetTurretRotations) {
        double turretDeltaRotations = targetTurretRotations - initialTurretRotation;
        return initialRotorRotation + getRotorDeltaRotations(turretDeltaRotations);
    }

    public double getRotorVelocityFromTurretVelocity(double turretRotationsPerSecond){
        return turretRotationsPerSecond / Constants.MOTOR_TO_TURRET_GEAR_RATIO;
    }

    public double getNearestTurretRotation(double clampedRotation) {
//        if (clampedRotation < 0.4 && clampedRotation > -0.4) {
//            return clampedRotation;
//        }
//        double currentTurretRotation = getTurretRotationFromRotorRotation();
//        if (currentTurretRotation * clampedRotation > 0) return clampedRotation;
//        if (currentTurretRotation < 0 && clampedRotation >= 0.4) {
//            return clampedRotation - 1;
//        } else if (currentTurretRotation > 0 && clampedRotation <= -0.4) {
//            return clampedRotation + 1;
//        } else {
//            return clampedRotation;
//        }
        return clampedRotation;
    }

    private void handleIdle() {
        //turretIO.setRotorRotationSetpoint(getRotorRotationsFromAbsoluteTurretRotation(IDLE_TURRET_ROTATION));
        turretIO.setVoltage(0);
    }

    private void handleScoring() {
        TurretSetpoint turretSetpoint = getTurretSetpoint();
        double nearestTurretRotation = getNearestTurretRotation(MathUtil.clamp(turretSetpoint.rotation, Constants.TURRET_MIN_ROTATIONS, Constants.TURRET_MAX_ROTATIONS));
        double rotationsPerSecondAdjusted = /*(
                MathUtil.isNear(getTurretRotationFromRotorRotation(), Constants.TURRET_MAX_ROTATIONS, Constants.ROTOR_ROTATION_TOLERANCE * Constants.MOTOR_TO_TURRET_GEAR_RATIO)
                || MathUtil.isNear(getTurretRotationFromRotorRotation(), Constants.TURRET_MIN_ROTATIONS, Constants.ROTOR_ROTATION_TOLERANCE * Constants.MOTOR_TO_TURRET_GEAR_RATIO)
        ) ? 0.0 : turretSetpoint.rotationsPerSecond;*/ 0.0;

        turretIO.setRotorRotationSetpoint(getRotorRotationsFromAbsoluteTurretRotation(nearestTurretRotation), getRotorVelocityFromTurretVelocity(rotationsPerSecondAdjusted));
    }

    private void handlePassing() {
        double nearestTurretRotation = getNearestTurretRotation(MathUtil.clamp(getTurretSetpoint().rotation, Constants.TURRET_MIN_ROTATIONS, Constants.TURRET_MAX_ROTATIONS));
        turretIO.setRotorRotationSetpoint(getRotorRotationsFromAbsoluteTurretRotation(nearestTurretRotation));
    }

    private void handleHubTrack() {
        double nearestTurretRotation = getNearestTurretRotation(MathUtil.clamp(getTurretSetpoint().rotation, Constants.TURRET_MIN_ROTATIONS, Constants.TURRET_MAX_ROTATIONS));
        turretIO.setRotorRotationSetpoint(getRotorRotationsFromAbsoluteTurretRotation(nearestTurretRotation));
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    private TurretSetpoint _getTurretSetpoint() {
        return switch (systemState) {
            case IDLE -> IDLE_TURRET_ROTATION;
            case SCORING -> {
                var currentScoringState = state.calculateScoringState(this);
                var nextScoringState = state.calculateNextScoringState(this);

                double rotations = currentScoringState.turretAngle().in(Rotations) - state.getEstimatedPose().getRotation().getRotations();
                double rotationsPerSecond = (nextScoringState.turretAngle().in(Rotations) - state.calculateNextPose(this).getRotation().getRotations()) - rotations;
                rotationsPerSecond = rotationsPerSecond / 0.02;

                yield SCORING_TURRET_ROTATION = new TurretSetpoint(rotations, rotationsPerSecond);
            }
            case PASSING -> PASSING_TURRET_ROTATION =
                    new TurretSetpoint(
                            state.calculatePassingState(this).turretAngle().in(Rotations) - state.getEstimatedPose().getRotation().getRotations(),
                            -state.getGyroRotationsPerSecond());
            case HUB_TRACK -> HUB_TRACK_TURRET_ROTATION =
                    new TurretSetpoint(
                            MathUtil.inputModulus(state.getTurretToHubFieldRelative().getRotations(), 0, 1) - state.getEstimatedPose().getRotation().getRotations(),
                            0);
        };
    }

    public TurretSetpoint getTurretSetpoint() {
        TurretSetpoint turretSetpoint = _getTurretSetpoint();
        return new TurretSetpoint(MathUtil.inputModulus(turretSetpoint.rotation, 0, 1), turretSetpoint.rotationsPerSecond);
    }

    public double getRotorSetpoint() {
        return getRotorRotationsFromAbsoluteTurretRotation(getTurretSetpoint().rotation);
    }

    @AutoLogOutput(key = "Turret At Setpoint")
    public boolean atSetpoint() {
        return MathUtil.isNear(getRotorSetpoint(), turretInputs.motorPositionRotations, Constants.ROTOR_ROTATION_TOLERANCE)
                || MathUtil.isNear(getRotorSetpoint() + 1.0 / Constants.MOTOR_TO_TURRET_GEAR_RATIO, turretInputs.motorPositionRotations, Constants.ROTOR_ROTATION_TOLERANCE)
                || MathUtil.isNear(getRotorSetpoint() - 1.0 / Constants.MOTOR_TO_TURRET_GEAR_RATIO, turretInputs.motorPositionRotations, Constants.ROTOR_ROTATION_TOLERANCE);
    }

    protected void setVoltage(Voltage volts){
        turretIO.setVoltage(volts.in(Volts));
    }

    @AutoLogOutput(key = "CRT Turret Position")
    private double CRT() {
        return getTurretRotation(leftEncoderInputs.positionRotations, rightEncoderInputs.positionRotations);
    }

    public record TurretSetpoint(double rotation, double rotationsPerSecond){}

    @AutoLogOutput(key = "Scoring Turret Rotation")
    private double getScoringTurretRotation(){
        return SCORING_TURRET_ROTATION.rotation;
    }
}
