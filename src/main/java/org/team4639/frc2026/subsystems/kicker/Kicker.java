/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.kicker;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;
import org.team4639.lib.util.LoggedTunableNumber;

public class Kicker extends FullSubsystem {
  private final RobotState state;
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  private final double KICK_RPM = 500 * 2 * 1.75;
  private final double IDLE_RPM = 0;

  @Getter private final KickerSysID sysID = new KickerSysID.KickerSysIDWPI(this, inputs);

  public enum WantedState {
    IDLE,
    KICK
  }

  public enum SystemState {
    IDLE,
    KICK
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  public final Subsystem dummy = new Subsystem() {};

  public Kicker(KickerIO io, RobotState state) {
    this.io = io;
    this.state = state;

    Logger.recordOutput("Kicker/SystemState", systemState.toString());
    this.setDefaultCommand(this.run(this::runStateMachine));
  }

  @Override
  public void periodicBeforeScheduler() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  @Override
  public void periodic() {

    LoggedTunableNumber.ifChanged(
        hashCode(),
        io::applyNewGains,
        PIDs.kickerKP,
        PIDs.kickerKI,
        PIDs.kickerKD,
        PIDs.kickerKS,
        PIDs.kickerKV,
        PIDs.kickerKA);
  }

  @Override
  public void periodicAfterScheduler() {
    state.setKickerStates(new Pair<>(this.wantedState, this.systemState));

    state.acceptCANMeasurement(inputs.connected);
    state.acceptTemperatureMeasurement(inputs.celsius);
  }

  private void runStateMachine() {
    SystemState newState = handleStateTransitions();
    if (newState != systemState) {
      Logger.recordOutput("Kicker/SystemState", newState.toString());
      systemState = newState;
    }

    if (DriverStation.isDisabled()) {
      systemState = SystemState.IDLE;
    }

    switch (systemState) {
      case IDLE:
        handleIdle();
        break;
      case KICK:
        handleKick();
        break;
    }
  }

  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case KICK -> SystemState.KICK;
    };
  }

  private void handleIdle() {
    io.setRotorVelocityRPM(IDLE_RPM);
  }

  private void handleKick() {
    io.setRotorVelocityRPM(KICK_RPM);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  protected void setVoltage(Voltage volts) {
    io.setVoltage(volts.in(Volts));
  }
}
