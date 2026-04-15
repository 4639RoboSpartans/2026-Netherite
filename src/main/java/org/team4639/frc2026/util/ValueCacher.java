/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.util;

import java.util.HashSet;
import java.util.function.Supplier;

public class ValueCacher<T, R> {
    private final HashSet<T> beenUpdated = new HashSet<>();
    private final Supplier<R> calculateValue;
    private R lastCalculatedValue;

    public ValueCacher(Supplier<R> calculateValueFunction) {
        this.calculateValue = calculateValueFunction;
    }

    public R get(T caller) {
        boolean needsToRecalculate = lastCalculatedValue == null || beenUpdated.contains(caller);

        if (needsToRecalculate) {
            this.lastCalculatedValue = calculateValue.get();
            beenUpdated.clear();
        }

        beenUpdated.add(caller);

        return this.lastCalculatedValue;
    }
}
