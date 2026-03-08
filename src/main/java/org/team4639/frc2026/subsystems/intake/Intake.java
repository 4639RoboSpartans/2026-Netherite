/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intake;

import edu.wpi.first.math.Pair;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.lib.util.FullSubsystem;
import org.team4639.lib.util.LoggedTunableNumber;

public class Intake extends FullSubsystem {
  private final RobotState state;
  private final IntakeRollerIO rollerIO;
  private final IntakeRollerIOInputsAutoLogged rollerInputs = new IntakeRollerIOInputsAutoLogged();

  private final double INTAKE_SURFACE_VELOCITY_FEET_PER_SECOND = 28 * 5;

  @Getter
  private final IntakeRollerSysID rollerSysID =
      new IntakeRollerSysID.IntakeRollerSysIDWPI(this, rollerInputs);

  public enum WantedState {
    IDLE,
    INTAKE,
    OUTTAKE
  }

  public enum SystemState {
    IDLE,
    INTAKE,
    OUTTAKE
  }

  @Setter private WantedState wantedState = WantedState.IDLE;

  private SystemState systemState = SystemState.IDLE;

  public Intake(IntakeRollerIO rollerIO, RobotState state) {
    this.rollerIO = rollerIO;
    this.state = state;
    rollerIO.updateInputs(rollerInputs);
    setDefaultCommand(run(this::runStateMachine));

    Logger.recordOutput("Intake/SystemState", systemState.toString());
  }

  @Override
  public void periodicBeforeScheduler() {
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake Rollers", rollerInputs);
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        rollerIO::applyNewGains,
        PIDs.rollerkP,
        PIDs.rollerkI,
        PIDs.rollerkD,
        PIDs.rollerkS,
        PIDs.rollerkV,
        PIDs.rollerkA);
  }

  @Override
  public void periodicAfterScheduler() {
    state.setIntakeStates(new Pair<>(this.wantedState, this.systemState));

    state.acceptCANMeasurement(rollerInputs.connected);
    state.acceptTemperatureMeasurement(rollerInputs.temperature);
  }

  public SystemState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case INTAKE -> {
        if (state.getIntakeExtensionFraction() < 0.55 && state.useIntakeProtection()) {
          yield SystemState.IDLE;
        } else {
          yield SystemState.INTAKE;
        }
      }
      case OUTTAKE -> {
        if (state.getIntakeExtensionFraction() < 0.55) {
          yield SystemState.IDLE;
        } else {
          yield SystemState.OUTTAKE;
        }
      }
    };
  }

  public void handleIdle() {
    rollerIO.setSurfaceVelocityFeetPerSecond(0);
  }

  public void handleIntaking() {
    rollerIO.setSurfaceVelocityFeetPerSecond(INTAKE_SURFACE_VELOCITY_FEET_PER_SECOND);
  }

  public void handleOuttaking() {
    rollerIO.setSurfaceVelocityFeetPerSecond(-INTAKE_SURFACE_VELOCITY_FEET_PER_SECOND);
  }

  public void setRollerVoltage(double volts) {
    rollerIO.setVoltage(volts);
  }

  private void runStateMachine() {
    SystemState newState = handleStateTransitions();
    if (newState != systemState) {
      Logger.recordOutput("Intake/SystemState", newState.toString());
      systemState = newState;
    }

    switch (systemState) {
      case IDLE -> handleIdle();
      case INTAKE -> handleIntaking();
      case OUTTAKE -> handleOuttaking();
    }
  }
}
