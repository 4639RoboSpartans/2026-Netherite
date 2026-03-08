/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.AbstractMap;

public class ShooterScoringData {
  /*private static final InterpolatingDoubleTreeMap scoringDistanceToRPM = InterpolatingDoubleTreeMap.ofEntries(
          new AbstractMap.SimpleImmutableEntry<>(1.97, 2500.0),
          new AbstractMap.SimpleImmutableEntry<>(2.15, 2550.0),
          new AbstractMap.SimpleImmutableEntry<>(2.45, 2600.0),
          new AbstractMap.SimpleImmutableEntry<>(2.69, 2650.0),
          new AbstractMap.SimpleImmutableEntry<>(2.88, 2700.0),
          new AbstractMap.SimpleImmutableEntry<>(3.10, 2800.0),
          new AbstractMap.SimpleImmutableEntry<>(3.30, 2900.0),
          new AbstractMap.SimpleImmutableEntry<>(3.54, 3000.0),
          new AbstractMap.SimpleImmutableEntry<>(3.77, 3100.0),
          new AbstractMap.SimpleImmutableEntry<>(3.99, 3175.0),
          new AbstractMap.SimpleImmutableEntry<>(4.26, 3250.0),
          new AbstractMap.SimpleImmutableEntry<>(4.52, 3300.0),
          new AbstractMap.SimpleImmutableEntry<>(4.73, 3350.0),
          new AbstractMap.SimpleImmutableEntry<>(4.90, 3450.0)
  )*/
  private static final InterpolatingDoubleTreeMap
      scoringDistanceToRPM =
          InterpolatingDoubleTreeMap.ofEntries(
              new AbstractMap.SimpleImmutableEntry<>(1.97, 2500.0),
              new AbstractMap.SimpleImmutableEntry<>(2.15, 2550.0),
              new AbstractMap.SimpleImmutableEntry<>(2.29, 2575.0),
              new AbstractMap.SimpleImmutableEntry<>(2.47, 2650.0),
              new AbstractMap.SimpleImmutableEntry<>(2.75, 2750.0),
              new AbstractMap.SimpleImmutableEntry<>(2.97, 2800.0),
              new AbstractMap.SimpleImmutableEntry<>(3.33, 2900.0),
              new AbstractMap.SimpleImmutableEntry<>(3.66, 3000.0),
              new AbstractMap.SimpleImmutableEntry<>(4.0, 3000.0),
              new AbstractMap.SimpleImmutableEntry<>(4.25, 3100.0),
              new AbstractMap.SimpleImmutableEntry<>(4.43, 3200.0),
              new AbstractMap.SimpleImmutableEntry<>(4.69, 3300.0),
              new AbstractMap.SimpleImmutableEntry<>(4.90, 3325.0)),
      /*scoringDistanceToHoodAngle = InterpolatingDoubleTreeMap.ofEntries(
              new AbstractMap.SimpleImmutableEntry<>(1.97, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(2.15, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(2.45, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(2.69, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(2.88, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(3.10, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(3.30, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(3.54, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(3.77, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(3.99, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(4.26, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(4.52, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(4.73, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(4.90, 20.0 / 360.0)
      )*/
      scoringDistanceToHoodAngle =
          InterpolatingDoubleTreeMap.ofEntries(
              new AbstractMap.SimpleImmutableEntry<>(1.97, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(2.15, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(2.29, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(2.47, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(2.75, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(2.97, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(3.33, 20.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(3.66, 22.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(4.0, 23.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(4.25, 25.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(4.43, 25.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(4.69, 27.0 / 360.0),
              new AbstractMap.SimpleImmutableEntry<>(4.90, 30.0 / 360.0)),
      scoringDistanceToTOF =
          InterpolatingDoubleTreeMap.ofEntries(
              new AbstractMap.SimpleImmutableEntry<>(2.47, 0.9527272727),
              new AbstractMap.SimpleImmutableEntry<>(2.47, 0.9927272727),
              new AbstractMap.SimpleImmutableEntry<>(2.7, 1.016),
              new AbstractMap.SimpleImmutableEntry<>(2.97, 1.089411765),
              new AbstractMap.SimpleImmutableEntry<>(3.3, 1.1336666667),
              new AbstractMap.SimpleImmutableEntry<>(3.66, 1.271052632),
              new AbstractMap.SimpleImmutableEntry<>(4.0, 1.351052632),
              new AbstractMap.SimpleImmutableEntry<>(4.25, 1.169),
              new AbstractMap.SimpleImmutableEntry<>(4.45, 1.171428571),
              new AbstractMap.SimpleImmutableEntry<>(4.7, 1.2146666667),
              new AbstractMap.SimpleImmutableEntry<>(4.9, 1.147368421));

  public static final ShooterLookupTable shooterLookupTable =
      new ShooterLookupTable(
          scoringDistanceToRPM, scoringDistanceToHoodAngle, scoringDistanceToTOF);
}
