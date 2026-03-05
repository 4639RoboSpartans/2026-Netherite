/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.team4639.frc2026.FieldConstants;

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
    private static final InterpolatingDoubleTreeMap scoringDistanceToRPM = InterpolatingDoubleTreeMap.ofEntries(
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
            new AbstractMap.SimpleImmutableEntry<>(4.90, 3300.0)
    ),
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
            scoringDistanceToHoodAngle = InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(1.97, 20.0 / 360.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.15, 20.0 / 360.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.29, 20.0 / 360.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.47, 20.0 / 360.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.75, 20.0 / 360.0),
                    new AbstractMap.SimpleImmutableEntry<>(2.97, 20.0 / 360.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.33, 20.0 / 360.0),
                    new AbstractMap.SimpleImmutableEntry<>(3.66, 20.0 / 360.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.0, 23.0 / 360.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.25, 25.0 / 360.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.43, 25.0 / 360.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.69, 27.0 / 360.0),
                    new AbstractMap.SimpleImmutableEntry<>(4.90, 30.0 / 360.0)
            ),
            scoringDistanceToTOF = InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(1.97, Math.sqrt(2 * 1.97 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(2.15, Math.sqrt(2 * 2.15 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(2.45, Math.sqrt(2 * 2.45 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(2.69, Math.sqrt(2 * 2.69 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(2.88, Math.sqrt(2 * 2.88 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(3.10, Math.sqrt(2 * 3.10 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(3.30, Math.sqrt(2 * 3.30 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(3.54, Math.sqrt(2 * 3.54 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(3.77, Math.sqrt(2 * 3.77 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(3.99, Math.sqrt(2 * 3.99 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(4.26, Math.sqrt(2 * 4.26 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(4.52, Math.sqrt(2 * 4.52 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(4.73, Math.sqrt(2 * 4.73 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81)),
                    new AbstractMap.SimpleImmutableEntry<>(4.90, Math.sqrt(2 * 4.90 * (1/Math.sqrt(3)) - FieldConstants.Hub.innerHeight)/Math.sqrt(9.81))
            );

    public static final ShooterLookupTable shooterLookupTable = new ShooterLookupTable(
            scoringDistanceToRPM,
            scoringDistanceToHoodAngle,
            scoringDistanceToTOF
    );
}
