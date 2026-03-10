/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;
import org.team4639.lib.util.LoggedTunableNumber;

public class Hood extends FullSubsystem {
  private final RobotState state;
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private double PASSING_HOOD_ANGLE = Constants.HOOD_MIN_ANGLE_DEGREES;
  private final double IDLE_HOOD_ANGLE = Constants.HOOD_MIN_ANGLE_DEGREES;

  @AutoLogOutput(key = "Hood Setpoint Degrees")
  private double SCORING_HOOD_ANGLE = 0;

  private double HOME_VOLTAGE = -3;

  private final double HOOD_TOLERANCE_DEGREES = 5;

  @Setter private double MANUAL_HOOD_ANGLE = 20;

  @Getter private final HoodSysID sysID = new HoodSysID.HoodSysIDWPI(this, inputs);

  public enum WantedState {
    IDLE,
    SCORING,
    PASSING,
    MANUAL
  }

  public enum SystemState {
    HOME_DOWN,
    HOME_UP,
    IDLE,
    SCORING,
    PASSING,
    MANUAL
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.HOME_DOWN;

  private WantedState lastZeroedWantedState = wantedState;

  public Hood(HoodIO io, RobotState state) {
    this.io = io;
    this.state = state;

    this.setDefaultCommand(this.run(this::runStateMachine));

    Logger.recordOutput("Hood/SystemState", systemState.toString());
  }

  @Override
  public void periodicBeforeScheduler() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    state.updateShooterState(null, Degrees.of(inputs.degrees), null);
  }

  @Override
  public void periodic() {

    if (org.team4639.frc2026.Constants.tuningMode) {
      LoggedTunableNumber.ifChanged(
          hashCode(),
          io::applyNewGains,
          PIDs.hoodKp,
          PIDs.hoodKi,
          PIDs.hoodKd,
          PIDs.hoodKs,
          PIDs.hoodKv,
          PIDs.hoodKa,
          PIDs.hoodKpSim,
          PIDs.hoodKiSim,
          PIDs.hoodKdSim);
    }

    if (this.systemState != SystemState.HOME_UP && this.systemState != SystemState.HOME_DOWN) {
      if (Math.abs(this.inputs.amps) >= 19.0) {
        if (this.inputs.volts < 0) {
          io.setPosition(Constants.HOOD_MIN_ANGLE_DEGREES);
        } else {
          io.setPosition(Constants.HOOD_MIN_ANGLE_DEGREES + Constants.HOOD_RANGE_DEGREES);
        }
      }
    }
  }

  @Override
  public void periodicAfterScheduler() {
    state.setHoodStates(new Pair<>(wantedState, systemState));
    state.acceptCANMeasurement(inputs.connected);
    state.acceptTemperatureMeasurement(inputs.celsius);
  }

  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> {
        if (systemState == SystemState.HOME_DOWN) {
          if (Math.abs(inputs.amps) > 19.0) {
            io.setPosition(Constants.HOOD_MIN_ANGLE_DEGREES);
            lastZeroedWantedState = WantedState.IDLE;
            yield SystemState.IDLE;
          } else {
            yield SystemState.HOME_DOWN;
          }
        }

        if (systemState == SystemState.HOME_UP) {
          if (Math.abs(inputs.amps) > 19.0) {
            io.setPosition(Constants.HOOD_MIN_ANGLE_DEGREES + Constants.HOOD_RANGE_DEGREES);
            lastZeroedWantedState = WantedState.IDLE;
            yield SystemState.IDLE;
          } else {
            yield SystemState.HOME_UP;
          }
        }

        yield SystemState.IDLE;
      }
      case SCORING -> SystemState.SCORING;
      case PASSING -> SystemState.PASSING;
      case MANUAL -> SystemState.MANUAL;
    };
  }

  private void handleHomeDown() {
    io.setVoltage(HOME_VOLTAGE);
  }

  private void handleHomeUp() {
    io.setVoltage(-HOME_VOLTAGE);
  }

  private void handleManual() {
    io.setPosition(MANUAL_HOOD_ANGLE);
  }

  private void handleIdle() {
    getSetpointAngle();
    io.setSetpointDegrees(IDLE_HOOD_ANGLE);
  }

  private void handleScoring() {
    getSetpointAngle();
    io.setSetpointDegrees(SCORING_HOOD_ANGLE);
  }

  private void handlePassing() {
    getSetpointAngle();
    io.setSetpointDegrees(PASSING_HOOD_ANGLE);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  @Deprecated
  public void setWantedState(WantedState wantedState, double scoringAngleRotations) {
    setWantedState(wantedState);
    if (wantedState == WantedState.PASSING) {
      this.PASSING_HOOD_ANGLE = Rotations.of(scoringAngleRotations).in(Degrees);
      if (this.PASSING_HOOD_ANGLE
          >= Constants.HOOD_MIN_ANGLE_DEGREES + Constants.HOOD_RANGE_DEGREES - 1)
        this.systemState = SystemState.HOME_UP;
      else if (this.PASSING_HOOD_ANGLE <= Constants.HOOD_MIN_ANGLE_DEGREES + 1)
        this.systemState = SystemState.HOME_DOWN;
    } else {
      this.SCORING_HOOD_ANGLE = Rotations.of(scoringAngleRotations).in(Degrees);
      if (this.SCORING_HOOD_ANGLE
          >= Constants.HOOD_MIN_ANGLE_DEGREES + Constants.HOOD_RANGE_DEGREES - 1)
        this.systemState = SystemState.HOME_UP;
      else if (this.SCORING_HOOD_ANGLE <= Constants.HOOD_MIN_ANGLE_DEGREES + 1)
        this.systemState = SystemState.HOME_DOWN;
    }
  }

  /**
   * Should not be called in comp code. All usages of setVoltage() needed for comp should be called
   * internally.
   *
   * @param volts
   */
  public void setVoltage(Voltage volts) {
    io.setVoltage(volts.in(Volts));
  }

  public double getSetpointAngle() {
    return switch (systemState) {
      case SCORING -> SCORING_HOOD_ANGLE =
          state.calculateScoringState(this).hoodAngle().in(Degrees);
      case PASSING -> PASSING_HOOD_ANGLE =
          state.calculatePassingState(this).hoodAngle().in(Degrees);
      case MANUAL -> MANUAL_HOOD_ANGLE;
      default -> Constants.HOOD_MIN_ANGLE_DEGREES;
    };
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(getSetpointAngle(), inputs.degrees, HOOD_TOLERANCE_DEGREES);
  }

  private void runStateMachine() {
    SystemState newState = handleStateTransitions();
    if (newState != systemState) {
      Logger.recordOutput("Hood/SystemState", newState.toString());
      systemState = newState;
    }

    if (DriverStation.isDisabled()) {
      systemState = SystemState.IDLE;
    }

    switch (systemState) {
      case HOME_DOWN:
        handleHomeDown();
        break;
      case HOME_UP:
        handleHomeUp();
        break;
      case IDLE:
        handleIdle();
        break;
      case SCORING:
        handleScoring();
        break;
      case PASSING:
        handlePassing();
        break;
      case MANUAL:
        handleManual();
        break;
    }
  }
}
