/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.unit;

public class Units2 {
  public static DoubleUnitConverter inchesToMeters = DoubleUnitConverter.linear(0.0254);
  public static DoubleUnitConverter metersToInches = inchesToMeters.inverted();
  public static DoubleUnitConverter inchesToCentimeters = DoubleUnitConverter.linear(02.54);
  public static DoubleUnitConverter centimetersToInches = inchesToCentimeters.inverted();
  public static DoubleUnitConverter inchesToFeet = DoubleUnitConverter.linear(1 / 12);
  public static DoubleUnitConverter feetToInches = inchesToFeet.inverted();
  public static DoubleUnitConverter centimetersToMeters = DoubleUnitConverter.linear(1 / 100);
  public static DoubleUnitConverter radiansToDegrees = DoubleUnitConverter.linear(180 / Math.PI);
  public static DoubleUnitConverter degreesToRadians = radiansToDegrees.inverted();
  public static DoubleUnitConverter rotationsToRadians = DoubleUnitConverter.linear(2 * Math.PI);
  public static DoubleUnitConverter radiansToRotations = rotationsToRadians.inverted();
  public static DoubleUnitConverter poundsToKilograms = DoubleUnitConverter.linear(0.453592);
  public static DoubleUnitConverter kilogramsToPounds = poundsToKilograms.inverted();
}
