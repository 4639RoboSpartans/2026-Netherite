/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.turret;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;
import org.team4639.lib.util.LoggedTunableNumber;

public class Turret extends FullSubsystem {
  private final RobotState state;
  private final TurretIO turretIO;
  private final EncoderIO leftEncoderIO, rightEncoderIO;
  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
  private final EncoderIOInputsAutoLogged leftEncoderInputs = new EncoderIOInputsAutoLogged(),
      rightEncoderInputs = new EncoderIOInputsAutoLogged();

  private final TurretSetpoint IDLE_TURRET_ROTATION = new TurretSetpoint(0, 0);
  private TurretSetpoint SCORING_TURRET_ROTATION = new TurretSetpoint(0, 0);
  private TurretSetpoint PASSING_TURRET_ROTATION = new TurretSetpoint(0, 0);
  private TurretSetpoint HUB_TRACK_TURRET_ROTATION = new TurretSetpoint(0, 0);

  private double initialTurretRotation;
  private double initialRotorRotation;

  protected static final double ODOMETRY_FREQUENCY = CANBus.roboRIO().isNetworkFD() ? 250.0 : 100.0;

  protected static final ReentrantLock motorPositionLock = new ReentrantLock();

  @Getter private final TurretSysID sysID = new TurretSysID.TurretSysIDWPI(this, turretInputs);

  private final Debouncer turretRezeroDebouncer = new Debouncer(0.5);
  private final Queue<Double> CRTMeasurements = new LinkedList<>();

  private boolean turretConnectionActivated = false;
  private boolean turretConnectionBeenLost = false;
  private final Debouncer turretStopped = new Debouncer(0.5, DebounceType.kRising);

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
    HUB_TRACK,
    REZERO
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  public final Subsystem dummy = new Subsystem() {};

  public Turret(
      TurretIO turretIO, EncoderIO leftEncoderIO, EncoderIO rightEncoderIO, RobotState state) {
    this.turretIO = turretIO;
    this.leftEncoderIO = leftEncoderIO;
    this.rightEncoderIO = rightEncoderIO;
    this.state = state;
    turretIO.updateInputs(turretInputs);
    leftEncoderIO.updateInputs(leftEncoderInputs);
    rightEncoderIO.updateInputs(rightEncoderInputs);
    rezeroTurret();

    setDefaultCommand(this.run(this::runStateMachine));
    Logger.recordOutput("Turret/SystemState", systemState);

    SmartDashboard.putBoolean("Rezero Encoders", false);
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

    state.updateShooterState(Double.NaN, Double.MAX_VALUE, getTurretRotationFromRotorRotation());
  }

  @Override
  public void periodic() {
    if (org.team4639.frc2026.Constants.tuningMode) {
      LoggedTunableNumber.ifChanged(
          hashCode(),
          turretIO::applyNewGains,
          PIDs.turretKp,
          PIDs.turretKi,
          PIDs.turretKd,
          PIDs.turretKs,
          PIDs.turretKv,
          PIDs.turretKa,
          PIDs.turretKpSim,
          PIDs.turretKiSim,
          PIDs.turretKdSim,
          PIDs.turretKsSim,
          PIDs.turretKvSim,
          PIDs.turretKaSim);
    }

    if (SmartDashboard.getBoolean("Rezero Encoders", false)) {
      if (leftEncoderIO instanceof EncoderIOCANCoder) {
        ((EncoderIOCANCoder) leftEncoderIO)
            .setOffsetRotations(leftEncoderInputs.positionWithoutOffset);
      }

      if (rightEncoderIO instanceof EncoderIOCANCoder) {
        ((EncoderIOCANCoder) rightEncoderIO)
            .setOffsetRotations(rightEncoderInputs.positionWithoutOffset);
      }
    }
  }

  private void runStateMachine() {
    SystemState newState = handleStateTransitions();
    if (newState != systemState) {
      Logger.recordOutput("Turret/SystemState", newState);
      systemState = newState;
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
      case HUB_TRACK:
        handleHubTrack();
        break;
      case REZERO:
        handleRezero();
        break;
    }
  }

  @Override
  public void periodicAfterScheduler() {
    RobotState.getInstance().setTurretStates(new Pair<>(wantedState, systemState));

    state.acceptCANMeasurement(turretInputs.connected);
    state.acceptTemperatureMeasurement(turretInputs.celsius);

    if (turretInputs.connected) {
      turretConnectionActivated = true;
    }
    if (turretConnectionActivated && !turretInputs.connected) {
      turretConnectionBeenLost = true;
    }
  }

  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> {
        if (systemState == SystemState.IDLE
            && turretConnectionActivated
            && turretConnectionBeenLost
            && turretInputs.connected) {
          yield SystemState.REZERO;
        } else if (systemState == SystemState.REZERO) {
          if (turretStopped.calculate(MathUtil.isNear(0, turretInputs.rotationsPerSecond, 1e-5))) {
            rezeroTurret();
            turretConnectionActivated = true;
            turretConnectionBeenLost = false;
            yield SystemState.IDLE;
          } else {
            yield SystemState.REZERO;
          }
        } else {
          yield SystemState.IDLE;
        }
      }
      case SCORING -> {
        if (systemState == SystemState.SCORING
            && turretConnectionActivated
            && turretConnectionBeenLost
            && turretInputs.connected) {
          yield SystemState.REZERO;
        } else if (systemState == SystemState.REZERO) {
          if (turretStopped.calculate(MathUtil.isNear(0, turretInputs.rotationsPerSecond, 1e-5))) {
            rezeroTurret();
            turretConnectionActivated = true;
            turretConnectionBeenLost = false;
            yield SystemState.SCORING;
          } else {
            yield SystemState.REZERO;
          }
        } else {
          yield SystemState.SCORING;
        }
      }
      case PASSING -> {
        if (systemState == SystemState.PASSING
            && turretConnectionActivated
            && turretConnectionBeenLost
            && turretInputs.connected) {
          yield SystemState.REZERO;
        } else if (systemState == SystemState.REZERO) {
          if (turretStopped.calculate(MathUtil.isNear(0, turretInputs.rotationsPerSecond, 1e-5))) {
            rezeroTurret();
            turretConnectionActivated = true;
            turretConnectionBeenLost = false;
            yield SystemState.PASSING;
          } else {
            yield SystemState.REZERO;
          }
        } else {
          yield SystemState.PASSING;
        }
      }
      case HUB_TRACK -> {
        if (systemState == SystemState.HUB_TRACK
            && turretConnectionActivated
            && turretConnectionBeenLost
            && turretInputs.connected) {
          yield SystemState.REZERO;
        } else if (systemState == SystemState.REZERO) {
          if (turretStopped.calculate(MathUtil.isNear(0, turretInputs.rotationsPerSecond, 1e-5))) {
            rezeroTurret();
            turretConnectionActivated = true;
            turretConnectionBeenLost = false;
            yield SystemState.HUB_TRACK;
          } else {
            yield SystemState.REZERO;
          }
        } else {
          yield SystemState.HUB_TRACK;
        }
      }
    };
  }

  // CRT, should only be used on startup to seed position and not while turret is moving
  public double getTurretRotation(double leftEncoderRotations, double rightEncoderRotations) {
    double abs1 = MathUtil.inputModulus(leftEncoderRotations, 0.0, 1.0) % 1;
    double abs2 = MathUtil.inputModulus(rightEncoderRotations, 0.0, 1.0) % 1;
    double ratio1 =
        Constants.SHARED_GEAR_TO_TURRET_GEAR_RATIO
            * Constants.SHARED_GEAR_TEETH
            / Constants.LEFT_ENCODER_GEAR_TEETH;
    double ratio2 =
        Constants.SHARED_GEAR_TO_TURRET_GEAR_RATIO
            * Constants.SHARED_GEAR_TEETH
            / Constants.RIGHT_ENCODER_GEAR_TEETH;
    double bestErr = Double.MAX_VALUE;
    double bestRot = Double.NaN;

    double nMinD =
        Math.min(
                ratio1 * Constants.TURRET_EXTENDED_MIN_ROTATIONS,
                ratio1 * Constants.TURRET_EXTENDED_MAX_ROTATIONS)
            - abs1;
    double nMaxD =
        Math.max(
                ratio1 * Constants.TURRET_EXTENDED_MIN_ROTATIONS,
                ratio1 * Constants.TURRET_EXTENDED_MAX_ROTATIONS)
            - abs1;
    int minN = (int) Math.floor(nMinD) - 1;
    int maxN = (int) Math.ceil(nMaxD) + 1;

    for (int n = minN; n <= maxN; n++) {
      double mechRot = (abs1 + n) / ratio1;
      if (mechRot < Constants.TURRET_EXTENDED_MIN_ROTATIONS - 1e-6
          || mechRot > Constants.TURRET_EXTENDED_MAX_ROTATIONS + 1e-6) {
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
    return getTurretRotationFromRotorRotation(turretInputs.rotations);
  }

  private double getTurretRotationFromRotorRotation(double rotorRotation) {
    return initialTurretRotation
        + ((rotorRotation - initialRotorRotation) * Constants.MOTOR_TO_TURRET_GEAR_RATIO);
  }

  public double getRotorDeltaRotations(double turretDeltaRotations) {
    return turretDeltaRotations / Constants.MOTOR_TO_TURRET_GEAR_RATIO;
  }

  public double getRotorRotationsFromAbsoluteTurretRotation(double targetTurretRotations) {
    double turretDeltaRotations = targetTurretRotations - initialTurretRotation;
    return initialRotorRotation + getRotorDeltaRotations(turretDeltaRotations);
  }

  public double getRotorVelocityFromTurretVelocity(double turretRotationsPerSecond) {
    return turretRotationsPerSecond / Constants.MOTOR_TO_TURRET_GEAR_RATIO;
  }

  public double getNearestTurretRotation(double clampedRotation) {
    return clampedRotation;
  }

  private void handleIdle() {
    turretIO.setVoltage(0);
  }

  private void handleScoring() {
    TurretSetpoint turretSetpoint = getTurretSetpoint();
    double nearestTurretRotation =
        getNearestTurretRotation(
            MathUtil.inputModulus(
                turretSetpoint.rotation,
                Constants.TURRET_MIN_ROTATIONS,
                Constants.TURRET_MAX_ROTATIONS));
    SmartDashboard.putNumber("turret target", nearestTurretRotation);
    double rotationsPerSecondAdjusted = turretSetpoint.rotationsPerSecond;

    if (Math.signum(nearestTurretRotation - getTurretRotationFromRotorRotation())
            != rotationsPerSecondAdjusted
        && turretIO instanceof TurretIOTalonFX) {
      ((TurretIOTalonFX) turretIO)
          .setRotorRotationSetpointSlot1(
              getRotorRotationsFromAbsoluteTurretRotation(nearestTurretRotation),
              getRotorVelocityFromTurretVelocity(rotationsPerSecondAdjusted));
    } else {

      turretIO.setRotorRotationSetpoint(
          getRotorRotationsFromAbsoluteTurretRotation(nearestTurretRotation),
          getRotorVelocityFromTurretVelocity(rotationsPerSecondAdjusted));
    }
  }

  private void handlePassing() {
    TurretSetpoint turretSetpoint = getTurretSetpoint();
    double nearestTurretRotation =
        getNearestTurretRotation(
            MathUtil.inputModulus(
                turretSetpoint.rotation,
                Constants.TURRET_MIN_ROTATIONS,
                Constants.TURRET_MAX_ROTATIONS));
    double rotationsPerSecondAdjusted = turretSetpoint.rotationsPerSecond;

    turretIO.setRotorRotationSetpoint(
        getRotorRotationsFromAbsoluteTurretRotation(nearestTurretRotation),
        getRotorVelocityFromTurretVelocity(rotationsPerSecondAdjusted));
  }

  private void handleHubTrack() {
    TurretSetpoint turretSetpoint = getTurretSetpoint();
    double nearestTurretRotation =
        getNearestTurretRotation(
            MathUtil.inputModulus(
                turretSetpoint.rotation,
                Constants.TURRET_MIN_ROTATIONS,
                Constants.TURRET_MAX_ROTATIONS));
    double rotationsPerSecondAdjusted = turretSetpoint.rotationsPerSecond;

    turretIO.setRotorRotationSetpoint(
        getRotorRotationsFromAbsoluteTurretRotation(nearestTurretRotation),
        getRotorVelocityFromTurretVelocity(rotationsPerSecondAdjusted));
  }

  private void handleRezero() {
    turretIO.setVoltage(0);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  private TurretSetpoint _getTurretSetpoint() {
    return switch (systemState) {
      case IDLE, REZERO -> IDLE_TURRET_ROTATION;
      case SCORING -> {
        var currentScoringState = state.calculateScoringState(this);
        var nextScoringState = state.calculateNextScoringState(this);

        double rotations =
            currentScoringState.turretRotations()
                - state.getSecondaryEstimatedPose().getRotation().getRotations();
        double rotationsPerSecond =
            (nextScoringState.turretRotations()
                    - state.calculateNextPose(this).getRotation().getRotations())
                - rotations;
        rotationsPerSecond = Constants.TURRET_FUDGE_SCALAR * rotationsPerSecond / 0.02;
        SmartDashboard.putNumber("turret target", rotationsPerSecond);

        yield SCORING_TURRET_ROTATION = new TurretSetpoint(rotations, rotationsPerSecond);
      }
      case PASSING -> {
        var currentPassingState = state.calculatePassingState(this);
        var nextPassingState = state.calculateNextPassingState(this);

        double rotations =
            currentPassingState.turretRotations()
                - state.getSecondaryEstimatedPose().getRotation().getRotations();
        double rotationsPerSecond =
            (nextPassingState.turretRotations()
                    - state.calculateNextPose(this).getRotation().getRotations())
                - rotations;
        rotationsPerSecond = Constants.TURRET_FUDGE_SCALAR * rotationsPerSecond / 0.02;

        yield PASSING_TURRET_ROTATION = new TurretSetpoint(rotations, rotationsPerSecond);
      }
      case HUB_TRACK -> HUB_TRACK_TURRET_ROTATION =
          new TurretSetpoint(
              MathUtil.inputModulus(
                      state.getBestHubTrackFieldRelative().getRotations(),
                      Constants.TURRET_MIN_ROTATIONS,
                      Constants.TURRET_MAX_ROTATIONS)
                  - state.getSecondaryEstimatedPose().getRotation().getRotations(),
              Constants.TURRET_FUDGE_SCALAR * -state.getGyroRotationsPerSecond());
    };
  }

  public TurretSetpoint getTurretSetpoint() {
    TurretSetpoint turretSetpoint = _getTurretSetpoint();
    return new TurretSetpoint(
        MathUtil.inputModulus(
            turretSetpoint.rotation,
            Constants.TURRET_MIN_ROTATIONS,
            Constants.TURRET_MAX_ROTATIONS),
        turretSetpoint.rotationsPerSecond);
  }

  public double getRotorSetpoint() {
    return getRotorRotationsFromAbsoluteTurretRotation(getTurretSetpoint().rotation);
  }

  @AutoLogOutput(key = "Turret At Setpoint")
  public boolean atSetpoint() {
    return (MathUtil.isNear(
                getRotorSetpoint(), turretInputs.rotations, Constants.ROTOR_ROTATION_TOLERANCE)
            || MathUtil.isNear(
                getRotorSetpoint() + 1.0 / Constants.MOTOR_TO_TURRET_GEAR_RATIO,
                turretInputs.rotations,
                Constants.ROTOR_ROTATION_TOLERANCE)
            || MathUtil.isNear(
                getRotorSetpoint() - 1.0 / Constants.MOTOR_TO_TURRET_GEAR_RATIO,
                turretInputs.rotations,
                Constants.ROTOR_ROTATION_TOLERANCE))
        && getTurretSetpoint().rotation < Constants.TURRET_MAX_ROTATIONS
        && getTurretSetpoint().rotation > Constants.TURRET_MIN_ROTATIONS;
  }

  protected void setVoltage(Voltage volts) {
    turretIO.setVoltage(volts.in(Volts));
  }

  @AutoLogOutput(key = "CRT Turret Position")
  private double CRT() {
    return getTurretRotation(
        leftEncoderInputs.positionRotations, rightEncoderInputs.positionRotations);
  }

  public record TurretSetpoint(double rotation, double rotationsPerSecond) {}

  @AutoLogOutput(key = "Scoring Turret Rotation")
  private double getScoringTurretRotation() {
    return SCORING_TURRET_ROTATION.rotation;
  }

  public void rezeroTurret() {
    initialTurretRotation =
        getTurretRotation(
            leftEncoderInputs.positionRotations, rightEncoderInputs.positionRotations);
    initialRotorRotation = turretInputs.rotations;

    if (turretIO instanceof TurretIOTalonFX) {
      // TODO: if the turret ever goes above 360 degrees of rotation
      ((TurretIOTalonFX) turretIO)
          .setSoftwareLimits(
              getRotorRotationsFromAbsoluteTurretRotation(Constants.TURRET_MAX_ROTATIONS),
              getRotorRotationsFromAbsoluteTurretRotation(Constants.TURRET_MIN_ROTATIONS));
    }
  }
}
