/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.spindexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;

public class Spindexer extends FullSubsystem {
  private final RobotState state;
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private final double SPIN_RPM = -50;
  private final double IDLE_RPM = 0;

  private double unjamStartTime = Double.NaN;
  private final double unjamTimePeriod = 0.2;

  @Getter private final SpindexerSysID sysID = new SpindexerSysID.SpindexerSysIDWPI(this, inputs);

  public final Subsystem dummy = new Subsystem() {};

  public enum WantedState {
    IDLE,
    SPIN
  }

  public enum SystemState {
    IDLE,
    SPIN,
    UNJAM
  }

  private WantedState wantedState = WantedState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  public Spindexer(SpindexerIO io, RobotState state) {
    this.io = io;
    this.state = state;

    Logger.recordOutput("Spindexer/SystemState", systemState.toString());
    this.setDefaultCommand(this.run(this::runStateMachine));
  }

  @Override
  public void periodicBeforeScheduler() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }

  @Override
  public void periodic() {}

  @Override
  public void periodicAfterScheduler() {
    state.setSpindexerStates(new Pair<>(this.wantedState, this.systemState));
    state.acceptCANMeasurement(inputs.connected);
    state.acceptTemperatureMeasurement(inputs.celsius);
  }

  private void runStateMachine() {
    SystemState newState = handleStateTransitions();
    if (newState != systemState) {
      Logger.recordOutput("Spindexer/SystemState", newState.toString());
      systemState = newState;
    }

    if (DriverStation.isDisabled()) {
      systemState = SystemState.IDLE;
    }

    switch (systemState) {
      case IDLE:
        handleIdle();
        break;
      case SPIN:
        handleKick();
        break;
      case UNJAM:
        handleUnjam();
        break;
    }
  }

  private SystemState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case SPIN -> {
        switch (systemState) {
          case IDLE -> {
            yield SystemState.SPIN;
          }
          case SPIN -> {
            if (Math.abs(inputs.amps) > 70) {
              unjamStartTime = Timer.getTimestamp();
              yield SystemState.UNJAM;
            } else {
              yield SystemState.SPIN;
            }
          }
          case UNJAM -> {
            if (Timer.getTimestamp() - unjamStartTime >= unjamTimePeriod) {
              yield SystemState.SPIN;
            } else {
              yield SystemState.IDLE;
            }
          }
        }
        yield SystemState.SPIN;
      }
    };
  }

  private void handleIdle() {
    io.setRotorVelocityRPM(IDLE_RPM);
  }

  private void handleKick() {
    io.setRotorVelocityRPM(SPIN_RPM);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  protected void setVoltage(Voltage volts) {
    io.setVoltage(volts.in(Volts));
  }

  private void handleUnjam() {
    io.setRotorVelocityRPM(-SPIN_RPM);
  }
}
