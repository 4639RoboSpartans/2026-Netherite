/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.tunable;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;
import org.team4639.lib.functional.DoubleConsumer2;
import org.team4639.lib.functional.DoubleSupplier2;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 *
 * @author elliot
 */
public class TunableNumber implements Sendable, DoubleSupplier2, Supplier<Double> {
    private double value;
    private boolean hasChanged = false;

    public TunableNumber(double value) {
        this.value = value;
    }

    public TunableNumber() {
        this(0);
    }

    private DoubleConsumer sendableConsumer = new DoubleConsumer2() {

        /**
         * Performs this operation on the given argument.
         *
         * @param value the input argument
         */
        @Override
        public void accept(double value) {
            hasChanged = TunableNumber.this.value != value;
            TunableNumber.this.value = value;
        }
    };

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.addDoubleProperty("Value", () -> value, sendableConsumer);
    }

    public void addDoubleConsumer(DoubleConsumer consumer) {
        sendableConsumer = sendableConsumer.andThen(consumer);
    }

    /**
     * Sends this TunableNumber to SmartDashboard.
     *
     * @param name
     */
    public TunableNumber send(String name) {
        SmartDashboard.putData(name, this);
        return this;
    }

    /**
     * Gets a result.
     *
     * @return a result
     */
    @Override
    public double getAsDouble() {
        return value;
    }

    /**
     * Gets a result.
     *
     * @return a result
     */
    @Override
    public Double get() {
        return value;
    }

    public boolean hasChanged() {
        return hasChanged;
    }
}
