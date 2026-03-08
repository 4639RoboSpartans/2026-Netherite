/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.unit;

import java.util.function.DoubleUnaryOperator;

public interface DoubleUnitConverter {
  double convert(double amountA);

  double convertBackwards(double amountB);

  /** Equivalent to {@link DoubleUnitConverter#compose DoubleUnitConverter.compose(this, next)}. */
  default DoubleUnitConverter then(DoubleUnitConverter next) {
    return compose(this, next);
  }

  /** Equivalent to {@link DoubleUnitConverter#invert DoubleUnitConverter.invert(this)}. */
  default DoubleUnitConverter inverted() {
    return invert(this);
  }

  /**
   * Returns the inverse of a unit converter, swapping {@link DoubleUnitConverter#convert} and
   * {@link DoubleUnitConverter#convertBackwards}
   */
  static DoubleUnitConverter invert(DoubleUnitConverter converter) {
    return new DoubleUnitConverter() {
      @Override
      public double convert(double amountA) {
        return converter.convertBackwards(amountA);
      }

      @Override
      public double convertBackwards(double amountB) {
        return converter.convert(amountB);
      }
    };
  }

  /** Composes unit converters. */
  static DoubleUnitConverter compose(
      DoubleUnitConverter converter1, DoubleUnitConverter converter2) {
    return new DoubleUnitConverter() {
      @Override
      public double convert(double amountA) {
        return converter2.convert(converter1.convert(amountA));
      }

      @Override
      public double convertBackwards(double amountB) {
        return converter1.convertBackwards(converter2.convertBackwards(amountB));
      }
    };
  }

  /** Creates a unit converter between units that have a linear relationship */
  static DoubleUnitConverter linear(double factor) {
    return linear(factor, 0, false);
  }

  /**
   * Creates a unit converter between units that have a linear relationship, with an offset
   *
   * @param applyOffsetFirst If true, {@link DoubleUnitConverter#convert convert(x)} returns (x +
   *     offset) * factor, otherwise, it returns (x * factor) + offset
   */
  static DoubleUnitConverter linear(double factor, double offset, boolean applyOffsetFirst) {
    double actualOffset = applyOffsetFirst ? offset * factor : offset;

    return new DoubleUnitConverter() {
      @Override
      public double convert(double amountA) {
        return amountA * factor + actualOffset;
      }

      @Override
      public double convertBackwards(double amountB) {
        return (amountB - actualOffset) / factor;
      }
    };
  }

  /** Creates a unit converter that linearly scales a range (minA, maxA) to a range (minB, maxB). */
  static DoubleUnitConverter linearConvertingRange(
      double minA, double maxA, double minB, double maxB) {
    double factor = (maxB - minB) / (maxA - minA);
    return linear(factor, minB - factor * minA, false);
  }

  static DoubleUnitConverter create(DoubleUnaryOperator forwards, DoubleUnaryOperator backwards) {
    return new DoubleUnitConverter() {
      @Override
      public double convert(double amountA) {
        return forwards.applyAsDouble(amountA);
      }

      @Override
      public double convertBackwards(double amountB) {
        return backwards.applyAsDouble(amountB);
      }
    };
  }
}
