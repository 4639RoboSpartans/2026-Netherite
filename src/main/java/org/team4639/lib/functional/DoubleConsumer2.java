/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.functional;

import java.util.Objects;
import java.util.function.DoubleConsumer;

@FunctionalInterface
public interface DoubleConsumer2 extends DoubleConsumer {
  public default DoubleConsumer2 and(DoubleConsumer otherConsumer) {
    Objects.requireNonNull(otherConsumer);
    var self = this;
    return new DoubleConsumer2() {

      /**
       * Performs this operation on the given argument.
       *
       * @param value the input argument
       */
      @Override
      public void accept(double value) {
        self.accept(value);
        otherConsumer.accept(value);
      }
    };
  }
}
