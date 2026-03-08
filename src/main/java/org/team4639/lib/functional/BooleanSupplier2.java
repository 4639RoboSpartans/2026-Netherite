/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.functional;

import java.util.function.BooleanSupplier;

public interface BooleanSupplier2 extends BooleanSupplier {

  public static BooleanSupplier2 and(BooleanSupplier a, BooleanSupplier b) {
    return () -> a.getAsBoolean() && b.getAsBoolean();
  }

  public static BooleanSupplier2 or(BooleanSupplier a, BooleanSupplier b) {
    return () -> a.getAsBoolean() || b.getAsBoolean();
  }

  public static BooleanSupplier2 negate(BooleanSupplier a, BooleanSupplier b) {
    return () -> !a.getAsBoolean() && !b.getAsBoolean();
  }

  public default BooleanSupplier2 and(BooleanSupplier other) {
    return and(this, other);
  }

  public default BooleanSupplier2 or(BooleanSupplier other) {
    return or(this, other);
  }

  public default BooleanSupplier2 negate(BooleanSupplier other) {
    return negate(this, other);
  }
}
