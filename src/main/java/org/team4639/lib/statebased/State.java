/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.statebased;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class State {
  private final String name;
  Collection<Command> commandsList;
  Supplier<State> onEmergency;
  Supplier<State> onAccelerationLimit;
  Supplier<State> onTimeout;
  Map<BooleanSupplier, Supplier<State>> endConditions;
  private final MutTime timeout;
  private Trigger trigger;

  public State(String name) {
    this.name = name;
    commandsList = new ArrayList<>();
    onEmergency = () -> this;
    onAccelerationLimit = () -> this;
    onTimeout = () -> this;
    endConditions = new HashMap<>();
    timeout = Seconds.mutable(Integer.MAX_VALUE);
  }

  public String getName() {
    return name;
  }

  public State whileTrue(Command... commands) {
    commandsList.addAll(Arrays.asList(commands));
    return this;
  }

  public void scheduleCommands() {
    commandsList.forEach(CommandScheduler.getInstance()::schedule);
  }

  public void cancelCommands() {
    commandsList.forEach(Command::cancel);
  }

  public State onEmergency(Supplier<State> state) {
    onEmergency = state;
    return this;
  }

  public State onAccelerationLimit(Supplier<State> state) {
    onAccelerationLimit = state;
    return this;
  }

  public State withEndCondition(BooleanSupplier condition, Supplier<State> nextState) {
    endConditions.put(condition, nextState);
    return this;
  }

  /**
   * Triggers an end condition when a trigger *becomes* true.
   *
   * @param trigger
   * @param nextState
   * @return this
   */
  public State onTrigger(Trigger trigger, Supplier<State> nextState) {
    trigger
        .and(() -> StateMachine.getState() == this)
        .onTrue(new InstantCommand(() -> StateMachine.setState(nextState.get())));
    return this;
  }

  public State withDeadline(Command cmd, Supplier<State> nextState) {
    this.whileTrue(cmd);
    this.withEndCondition(() -> !cmd.isScheduled(), nextState);
    return this;
  }

  public State withTimeout(Time timeout, Supplier<State> nextState) {
    this.timeout.mut_replace(timeout);
    this.onTimeout = nextState;
    return this;
  }

  public void evaluate(double timeSinceLastStateChange) {
    for (BooleanSupplier b : endConditions.keySet()) {
      if (b.getAsBoolean()) {
        State next = endConditions.get(b).get();
        StateMachine.setState(next);
      }
    }
    if (timeSinceLastStateChange >= timeout.in(Seconds)) StateMachine.setState(onTimeout.get());
  }

  @Override
  public String toString() {
    return name;
  }

  public Trigger trigger() {
    return new Trigger(() -> StateMachine.getState() == this);
  }

  public Trigger getTrigger() {
    return trigger =
        Objects.requireNonNullElse(trigger, new Trigger(() -> StateMachine.getState() == this));
  }

  public Trigger and(BooleanSupplier other) {
    return getTrigger().and(other);
  }

  public State mapEndConditions(Map<BooleanSupplier, Supplier<State>> endConditions) {
    endConditions.forEach(this::withEndCondition);
    return this;
  }

  public State mapTriggerEndConditions(Map<Trigger, Supplier<State>> endConditions) {
    endConditions.forEach(this::onTrigger);
    return this;
  }

  public State mapTriggerCommandsWhileTrue(Map<Trigger, Command> triggerCommandMap) {
    triggerCommandMap.forEach((t, c) -> this.and(t).whileTrue(c));
    return this;
  }

  public State mapTriggerCommandsOnTrue(Map<Trigger, Command> triggerCommandMap) {
    triggerCommandMap.forEach((t, c) -> this.and(t).onTrue(c));
    return this;
  }
}
