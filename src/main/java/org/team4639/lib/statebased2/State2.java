/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.statebased2;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.AccessLevel;
import lombok.Getter;

@Getter(AccessLevel.PROTECTED)
public class State2 {
    private String name;
    private Set<Command> commandsWhileRunning;
    private Set<InstantCommand> commandsOnEnter;
    private Set<InstantCommand> commandsOnExit;
    private Map<BooleanSupplier, Supplier<State2>> endConditions;
    private StateMachine2 stateMachine;
    private boolean initialized = false;

    protected State2(String name, StateMachine2 stateMachine) {
        this.name = name;
        this.stateMachine = stateMachine;
        this.commandsWhileRunning = new HashSet<>();
        this.commandsOnEnter = new HashSet<>();
        this.commandsOnExit = new HashSet<>();
        this.endConditions = new HashMap<>();
    }

    /**
     * Adds commands to the set of commands that will run while this
     * state is active. If there have previously been commands configured to
     * run using this method, this will <b>not</b> override that and instead
     * will add the new commands to the set. There is no guarantee of the order
     * these commands will be scheduled/cancelled, except that they will all be scheduled
     * on the loop that this state becomes active and cancelled on the loop this state
     * becomes inactive.
     * @param commands The commands. All requirements of these commands must be contained in the
     * requirements of this state's parent state machine, or else a RuntimeException is thrown.
     * @return this object, for method chaining.
     */
    public State2 whileRunning(Command... commands) {
        var allowedRequirements = stateMachine.subsystems;
        if (!(Arrays.stream(commands)
                .allMatch(command -> allowedRequirements.containsAll(command.getRequirements())))) {
            throw new RuntimeException(
                    "Requirements of at least one command are not contained within the allowed requirements of " + name
                            + "'s parent state machine!");
        }
        Arrays.stream(commands).forEach(commandsWhileRunning::add);
        this.initialized = false;
        return this;
    }

    /**
     * Adds commands to the set of commands that will run when this state becomes
     * active. If there have previously been commands configured to
     * run using this method, this will <b>not</b> override that and instead
     * will add the new commands to the set. There is no guarantee of the order
     * these commands will be scheduled, except that they will all be scheduled
     * on the loop that this state becomes active.
     * <p>
     * This method only takes {@link InstantCommand}s to preserve the encapsulated
     * design of the state machine. For commands that should run for the entirety
     * of the state's active duration, see {@link State2#whileRunning(Command...)}
     * instead. If using this method to trigger changes in a Subsystem's internal state,
     * users may want to reset the Subsystem's internal state using {@link State2#onExit(InstantCommand...)}
     * @param commands The commands. All requirements of these commands must be contained in the
     * requirements of this state's parent state machine, or else a RuntimeException is thrown.
     * @return this object, for method chaining.
     */
    public State2 onEnter(InstantCommand... commands) {
        var allowedRequirements = stateMachine.subsystems;
        if (!(Arrays.stream(commands)
                .allMatch(command -> allowedRequirements.containsAll(command.getRequirements())))) {
            throw new RuntimeException(
                    "Requirements of at least one command are not contained within the allowed requirements of " + name
                            + "'s parent state machine!");
        }
        Arrays.stream(commands).forEach(commandsOnEnter::add);
        this.initialized = false;
        return this;
    }

    /**
     * Adds commands to the set of commands that will run when this state becomes
     * inactive. If there have previously been commands configured to
     * run using this method, this will <b>not</b> override that and instead
     * will add the new commands to the set. There is no guarantee of the order
     * these commands will be scheduled, except that they will all be scheduled
     * on the loop that this state becomes inactive.
     * <p>
     * This method only takes {@link InstantCommand}s to preserve the encapsulated
     * design of the state machine. For commands that should run for the entirety
     * of the state's active duration, see {@link State2#whileRunning(Command...)}
     * instead.
     * @param commands The commands. All requirements of these commands must be contained in the
     * requirements of this state's parent state machine, or else a RuntimeException is thrown.
     * @return this object, for method chaining.
     */
    public State2 onExit(InstantCommand... commands) {
        var allowedRequirements = stateMachine.subsystems;
        if (!(Arrays.stream(commands)
                .allMatch(command -> allowedRequirements.containsAll(command.getRequirements())))) {
            throw new RuntimeException(
                    "Requirements of at least one command are not contained within the allowed requirements of " + name
                            + "'s parent state machine!");
        }
        Arrays.stream(commands).forEach(commandsOnExit::add);
        return this;
    }

    /**
     * Adds an end condition to this state and supplies the next state to transition to.
     * @param endCondition a BooleanSupplier that returns true when this state should change to the next state.
     * This is simply queried to see if it <b>is</b> true, not if it becomes true to see if the state should change.
     * @param nextStateSupplier supplies the next state. Is a supplier to account for null errors if a state
     * has not been initialized yet, but suppliers should generally point to one {@link State2}. A RuntimeException
     * will be thrown by the parent {@link StateMachine2} if the supplied state does not belong to the parent machine.
     * @return this object, for method chaining.
     */
    public State2 withEndCondition(BooleanSupplier endCondition, Supplier<State2> nextStateSupplier) {
        this.endConditions.put(endCondition, nextStateSupplier);
        return this;
    }

    public State2 withDeadline(Command command, Supplier<State2> nextState) {
        return this.whileRunning(command).withEndCondition(() -> !command.isScheduled(), nextState);
    }

    public State2 onTrigger(Trigger trigger, Supplier<State2> nextStateSupplier) {
        new Trigger(new InternalTwoStateTrigger(new Trigger(() -> stateMachine.getActiveState() == this), trigger))
                .onTrue(new InstantCommand(() -> stateMachine.setState(nextStateSupplier.get())));
        return this;
    }

    protected void init() {
        commandsOnEnter.forEach(CommandScheduler.getInstance()::schedule);
        commandsWhileRunning.forEach(CommandScheduler.getInstance()::schedule);
        this.initialized = true;
    }

    protected void exit() {
        commandsWhileRunning.forEach(CommandScheduler.getInstance()::cancel);
        commandsOnExit.forEach(CommandScheduler.getInstance()::schedule);
        this.initialized = false;
    }

    /**
     * Because of Things, only returns true when it should *fire*, not
     * all the time it should be true.
     */
    private class InternalTwoStateTrigger implements BooleanSupplier {
        private final BooleanSupplier first;
        private final BooleanSupplier second;
        private boolean firstWasTrue = false;
        private boolean secondWasTrue = false;

        public InternalTwoStateTrigger(Trigger first, Trigger second) {
            this.first = first;
            this.second = second;
        }

        public boolean getAsBoolean() {
            boolean ret = firstWasTrue && !secondWasTrue && first.getAsBoolean() && second.getAsBoolean();

            firstWasTrue = first.getAsBoolean();
            secondWasTrue = second.getAsBoolean();

            return ret;
        }
    }
}