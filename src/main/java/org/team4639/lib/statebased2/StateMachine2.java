/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.statebased2;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;
import org.team4639.lib.util.VirtualSubsystem;

/**
 * V2 implementation of state machine
 */
public class StateMachine2 extends VirtualSubsystem {
    private State2 defaultState = null;
    private State2 currentState = null;
    private final Set<State2> allStates;
    protected final Set<Subsystem> subsystems;
    private final Map<String, UnaryOperator<State2>> templates;
    private String networkTablesKey = null;
    private Set<ActiveMode> activeModes = new HashSet<>();
    private Trigger isActive = new Trigger(() -> false);

    private final String DEFAULT_TEMPLATE_KEY = "DEFAULT";

    public StateMachine2(Subsystem... subsystems) {
        this.subsystems = new HashSet<>();
        Arrays.stream(subsystems).forEach(this.subsystems::add);
        allStates = new HashSet<>();
        templates = new HashMap<>();
    }

    /**
     * Creates a new state for use with this StateMachine. If there is
     * a default template specified, then the state will be created with
     * that template, otherwise it is created as a blank {@link State2}
     * with nothing except the name and this state machine configured.
     * @param name Name of the State
     * @return the state
     */
    public State2 state(String name) {
        var state = new State2(name, this);
        if (templates.containsKey(DEFAULT_TEMPLATE_KEY)) {
            state = templates.get(DEFAULT_TEMPLATE_KEY).apply(state);
        }
        allStates.add(state);
        return state;
    }

    /**
     * Creates a default state for use with this StateMachine. If there is
     * a default template specified, then the state will be created with
     * that template, otherwise it is created as a blank {@link State2}
     * with nothing except the name and this state machine configured.
     * @param name Name of the State
     * @return the state. This will be the starting state of the state
     * machine as well as the state to return to if its current state
     * ever becomes null.
     */
    public State2 defaultState(String name) {
        var state = state(name);
        this.defaultState = state;
        return state;
    }

    /**
     * Creates a new state for use with this StateMachine. If the template
     * given has been previously configured, the state will be created with
     * that template, otherwise a RuntimeException will be thrown.
     * @param templateName the name of the template.
     * @param name the name of the state.
     * @return the state.
     */
    public State2 state(String templateName, String name) {
        var state = new State2(name, this);
        if (templates.containsKey(templateName)) {
            state = templates.get(templateName).apply(state);
        } else {
            throw new RuntimeException("No template configured with name " + templateName + "!");
        }
        allStates.add(state);
        return state;
    }

    /**
     * Creates a default template for all states created by this state
     * machine. States created using {@link StateMachine2#state(String)}
     * will by default have this template.
     * @param function a function taking a blank {@link State2} and handling all the
     * configuration necessary for this template. If another default template was previously
     * set, this will override that template.
     * @return this object, for method chaining.
     */
    public StateMachine2 template(UnaryOperator<State2> function) {
        this.templates.put(DEFAULT_TEMPLATE_KEY, function);
        return this;
    }

    /**
     * Creates a template for states created by this state
     * machine. States created using {@link StateMachine2#state(String, String)}
     * where the first parameter matches the name of the template will be created
     * using this template.
     * @param templateName the name of this template.
     * @param function a function taking a blank {@link State2} and handling all the
     * configuration necessary for this template. If another template was previously
     * set using the same name, this will override that template.
     * @return this object, for method chaining.
     */
    public StateMachine2 template(String templateName, UnaryOperator<State2> function) {
        this.templates.put(templateName, function);
        return this;
    }

    /**
     * Handles default states. Does not need to be called by user code.
     */
    @Override
    public void periodic() {
        if (this.isActive.getAsBoolean()){
            // handle default state and state nulls
            if (defaultState == null) {
                throw new RuntimeException("Default state has not been configured!");
            } else if (currentState == null) {
                currentState = defaultState;
                currentState.init();
            }
        }
    }

    /**
     * Contains logic for handling states. As long as {@link VirtualSubsystem} is
     * implemented correctly this does not need to be called by user code.
     */
    @Override
    public void periodicAfterScheduler() {
        if (this.isActive.getAsBoolean()){
            if (!currentState.isInitialized()) {
                currentState.exit();
                currentState.init();
            }

            // check if we need to change state
            Optional<Entry<BooleanSupplier, Supplier<State2>>> validEndCondition =
                    currentState.getEndConditions().entrySet().stream()
                            .filter(entry -> entry.getKey().getAsBoolean())
                            .findFirst();
            // if it exists, handle the state change
            validEndCondition.ifPresent(entry -> {
                currentState.exit();
                var newState = entry.getValue().get();
                if (newState.getStateMachine() != this) {
                    currentState = defaultState;
                    currentState.init();
                    throw new RuntimeException(
                            "Parent state machine of " + newState.getName() + " does not match this state machine!");
                } else {
                    currentState = newState;
                    currentState.init();
                }
            });

            if (networkTablesKey != null) {
                Logger.recordOutput(networkTablesKey, currentState.getName());
            }
        }
    }

    public State2 getActiveState() {
        return this.currentState;
    }

    /**
     * Publishes the name of the current running state of this
     * state machine to NT.
     * @param key the key. The fully qualified name of the NetworkTables
     * path will be "/Internal/State/<key>"
     * @return this object, for method chaining.
     */
    public StateMachine2 publishToNT(String key) {
        this.networkTablesKey = "/Internal/State/" + key;
        return this;
    }

    protected void setState(State2 state) {
        this.currentState.exit();
        this.currentState = state;
        this.currentState.init();
    }

    @RequiredArgsConstructor
    public static enum ActiveMode{
        AUTO(RobotState::isAutonomous),
        TELEOP(RobotState::isTeleop),
        DISABLED(RobotState::isDisabled);

        private final BooleanSupplier active;
    }

    public StateMachine2 activeDuring(ActiveMode... modes){
        this.activeModes.addAll(Arrays.asList(modes));
        this.isActive = new Trigger(() -> activeModes.stream().anyMatch(mode -> mode.active.getAsBoolean()));
        this.isActive.onTrue(Commands.runOnce(() -> this.setState(this.defaultState)));
        return this;
    }
}
