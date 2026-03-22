/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.Constants;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;
import org.team4639.lib.util.LoggedTunableNumber;

public class Shooter extends FullSubsystem {
  private final RobotState state;
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double PASSING_RPM = 0;
  private final double IDLE_VOLTAGE = 0;

  @AutoLogOutput(key = "Shooter Scoring RPM")
  private double SCORING_RPM = 0;

  @Setter private double MANUAL_RPM = 0;

  private final double SHOOTING_RPM_TOLERANCE = 50;

  private final LoggedTunableNumber desiredRPM =
      new LoggedTunableNumber("Desired RPM").initDefault(0);

  @Getter private final ShooterSysID sysID = new ShooterSysID.ShooterSysIDWPI(this, inputs);

  public final Subsystem dummy = new Subsystem() {};

  public enum WantedState {
    OFF,
    IDLE,
    SCORING,
    PASSING,
    MANUAL
  }

  public enum SystemState {
    OFF,
    IDLE,
    SCORING,
    PASSING,
    MANUAL
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState systemState = SystemState.OFF;

  public Shooter(ShooterIO io, RobotState state) {
    this.io = io;
    this.state = state;

    this.setDefaultCommand(this.run(this::runStateMachine));
    Logger.recordOutput("Shooter/SystemState", systemState.toString());
  }

  @Override
  public void periodicBeforeScheduler() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    state.updateShooterState(inputs.leftRPM, Double.NaN, Double.NaN);
  }

  @Override
  public void periodic() {

    if (Constants.tuningMode) {
      LoggedTunableNumber.ifChanged(
          hashCode(),
          io::applyNewGains,
          PIDs.shooterKp,
          PIDs.shooterKi,
          PIDs.shooterKd,
          PIDs.shooterKs,
          PIDs.shooterKv,
          PIDs.shooterKa,
          PIDs.shooterKpSim,
          PIDs.shooterKiSim,
          PIDs.shooterKdSim,
          PIDs.shooterKsSim,
          PIDs.shooterKvSim,
          PIDs.shooterKaSim);
    }
  }

  @Override
  public void periodicAfterScheduler() {
    RobotState.getInstance().setShooterStates(new Pair<>(wantedState, systemState));

    state.acceptCANMeasurement(inputs.leftConnected);
    state.acceptCANMeasurement(inputs.rightConnected);
    state.acceptTemperatureMeasurement(inputs.leftCelsius);
    state.acceptTemperatureMeasurement(inputs.rightTemperature);
  }

  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case SCORING -> SystemState.SCORING;
      case PASSING -> SystemState.PASSING;
      case IDLE -> SystemState.IDLE;
      case MANUAL -> SystemState.MANUAL;
      default -> SystemState.OFF;
    };
  }

  private void handleOff() {
    io.setVoltage(0);
  }

  private void handleScoring() {
    io.setRPM(this.getSetpointRPM());
  }

  private void handlePassing() {
    io.setRPM(this.getSetpointRPM());
  }

  private void handleIdle() {
    io.setVoltage(IDLE_VOLTAGE);
  }

  private void handleManual() {
    io.setRPM(MANUAL_RPM);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  @Deprecated
  public void setWantedState(WantedState wantedState, double scoringRPM) {
    setWantedState(wantedState);
    if (wantedState == WantedState.PASSING) {
      this.PASSING_RPM = scoringRPM;
    } else {
      this.SCORING_RPM = scoringRPM;
    }
  }

  /**
   * Should not be called in comp code. All usages of setVoltage() needed for comp should be called
   * internally.
   *
   * @param volts voltage to set shooter motors to
   */
  protected void setVoltage(Voltage volts) {
    io.setVoltage(volts.in(Volts));
  }

  public double getSetpointRPM() {
    return switch (systemState) {
      case SCORING -> SCORING_RPM = state.calculateScoringState(this).shooterRPM();
      case PASSING -> PASSING_RPM = state.calculateScoringState(this).shooterRPM();
      case MANUAL -> MANUAL_RPM;
      default -> 0;
    };
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(getSetpointRPM(), -inputs.leftRPM, SHOOTING_RPM_TOLERANCE);
  }

  public boolean aboveSetpoint() {
    return Math.abs(getSetpointRPM()) < Math.abs(inputs.leftRPM);
  }

  private void runStateMachine() {
    SystemState newState = handleStateTransitions();
    if (newState != systemState) {
      Logger.recordOutput("Shooter/SystemState", newState.toString());
      systemState = newState;
    }

    if (DriverStation.isDisabled()) {
      systemState = SystemState.OFF;
    }

    switch (systemState) {
      case OFF:
        handleOff();
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
    }
  }
}
