/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.functional;

import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;

@FunctionalInterface
public interface DoubleSupplier2 extends DoubleSupplier {

  // Static helper methods for binary operations
  static double plus(DoubleSupplier first, DoubleSupplier second) {
    return first.getAsDouble() + second.getAsDouble();
  }

  static double minus(DoubleSupplier first, DoubleSupplier second) {
    return first.getAsDouble() - second.getAsDouble();
  }

  static double times(DoubleSupplier first, DoubleSupplier second) {
    return first.getAsDouble() * second.getAsDouble();
  }

  static double over(DoubleSupplier first, DoubleSupplier second) {
    return first.getAsDouble() / second.getAsDouble();
  }

  static double pow(DoubleSupplier first, DoubleSupplier second) {
    return Math.pow(first.getAsDouble(), second.getAsDouble());
  }

  static double max(DoubleSupplier first, DoubleSupplier second) {
    return Math.max(first.getAsDouble(), second.getAsDouble());
  }

  static double min(DoubleSupplier first, DoubleSupplier second) {
    return Math.min(first.getAsDouble(), second.getAsDouble());
  }

  // Static helper methods for unary operations
  static double sin(DoubleSupplier supplier) {
    return Math.sin(supplier.getAsDouble());
  }

  static double cos(DoubleSupplier supplier) {
    return Math.cos(supplier.getAsDouble());
  }

  static double tan(DoubleSupplier supplier) {
    return Math.tan(supplier.getAsDouble());
  }

  static double asin(DoubleSupplier supplier) {
    return Math.asin(supplier.getAsDouble());
  }

  static double acos(DoubleSupplier supplier) {
    return Math.acos(supplier.getAsDouble());
  }

  static double atan(DoubleSupplier supplier) {
    return Math.atan(supplier.getAsDouble());
  }

  static double abs(DoubleSupplier supplier) {
    return Math.abs(supplier.getAsDouble());
  }

  static double applyOperator(DoubleSupplier supplier, DoubleUnaryOperator operator) {
    return operator.applyAsDouble(supplier.getAsDouble());
  }

  public default DoubleSupplier2 plus(DoubleSupplier otherSupplier) {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.plus(self, otherSupplier);
      }
    };
  }

  public default DoubleSupplier2 minus(DoubleSupplier otherSupplier) {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.minus(self, otherSupplier);
      }
    };
  }

  public default DoubleSupplier2 times(DoubleSupplier otherSupplier) {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.times(self, otherSupplier);
      }
    };
  }

  public default DoubleSupplier2 over(DoubleSupplier otherSupplier) {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /*
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.over(self, otherSupplier);
      }
    };
  }

  public default DoubleSupplier2 pow(DoubleSupplier otherSupplier) {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.pow(self, otherSupplier);
      }
    };
  }

  public default DoubleSupplier2 max(DoubleSupplier otherSupplier) {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.max(self, otherSupplier);
      }
    };
  }

  public default DoubleSupplier2 min(DoubleSupplier otherSupplier) {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.min(self, otherSupplier);
      }
    };
  }

  public default DoubleSupplier2 sin() {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.sin(self);
      }
    };
  }

  public default DoubleSupplier2 cos() {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.cos(self);
      }
    };
  }

  public default DoubleSupplier2 tan() {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.tan(self);
      }
    };
  }

  public default DoubleSupplier2 asin() {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.asin(self);
      }
    };
  }

  public default DoubleSupplier2 acos() {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.acos(self);
      }
    };
  }

  public default DoubleSupplier2 atan() {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.atan(self);
      }
    };
  }

  public default DoubleSupplier2 abs() {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.abs(self);
      }
    };
  }

  public default DoubleSupplier2 thenApply(DoubleUnaryOperator operator) {
    final DoubleSupplier2 self = this; // Capture 'this'
    return new DoubleSupplier2() {
      /**
       * Gets a result.
       *
       * @return a result
       */
      @Override
      public double getAsDouble() {
        return DoubleSupplier2.applyOperator(self, operator);
      }
    };
  }
}
