/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import java.util.AbstractMap;

public class ShooterScoringData {
    private static final InterpolatingDoubleTreeMap scoringDistanceToRPM = InterpolatingDoubleTreeMap.ofEntries(
            new AbstractMap.SimpleImmutableEntry<>(1.0, 1043.0),
            new AbstractMap.SimpleImmutableEntry<>(2.0, 1163.5),
            new AbstractMap.SimpleImmutableEntry<>(3.0, 1287.0),
            new AbstractMap.SimpleImmutableEntry<>(4.0, 1405.5),
            new AbstractMap.SimpleImmutableEntry<>(5.0, 1518.0),
            new AbstractMap.SimpleImmutableEntry<>(6.0, 1624.0),
            new AbstractMap.SimpleImmutableEntry<>(7.0, 1724.5),
            new AbstractMap.SimpleImmutableEntry<>(8.0, 1819.5),
            new AbstractMap.SimpleImmutableEntry<>(9.0, 1911.0),
            new AbstractMap.SimpleImmutableEntry<>(10.0, 1998.0),
            new AbstractMap.SimpleImmutableEntry<>(11.0, 2082.0),
            new AbstractMap.SimpleImmutableEntry<>(12.0, 2163.0),
            new AbstractMap.SimpleImmutableEntry<>(13.0, 2241.0),
            new AbstractMap.SimpleImmutableEntry<>(14.0, 2316.0),
            new AbstractMap.SimpleImmutableEntry<>(15.0, 2389.5),
            new AbstractMap.SimpleImmutableEntry<>(16.0, 2460.5),
            new AbstractMap.SimpleImmutableEntry<>(17.0, 2529.5),
            new AbstractMap.SimpleImmutableEntry<>(18.0, 2596.5),
            new AbstractMap.SimpleImmutableEntry<>(19.0, 2662.5),
            new AbstractMap.SimpleImmutableEntry<>(20.0, 2726.5)
    ),
            scoringDistanceToHoodAngle = InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(1.0, 0.25 - 0.208),
                    new AbstractMap.SimpleImmutableEntry<>(2.0, 0.25 - 0.186),
                    new AbstractMap.SimpleImmutableEntry<>(3.0, 0.25 - 0.173),
                    new AbstractMap.SimpleImmutableEntry<>(4.0, 0.25 - 0.164),
                    new AbstractMap.SimpleImmutableEntry<>(5.0, 0.25 - 0.158),
                    new AbstractMap.SimpleImmutableEntry<>(6.0, 0.25 - 0.154),
                    new AbstractMap.SimpleImmutableEntry<>(7.0, 0.25 - 0.151),
                    new AbstractMap.SimpleImmutableEntry<>(8.0, 0.25 - 0.148),
                    new AbstractMap.SimpleImmutableEntry<>(9.0, 0.25 - 0.146),
                    new AbstractMap.SimpleImmutableEntry<>(10.0, 0.25 - 0.144),
                    new AbstractMap.SimpleImmutableEntry<>(11.0, 0.25 - 0.142),
                    new AbstractMap.SimpleImmutableEntry<>(12.0, 0.25 - 0.141),
                    new AbstractMap.SimpleImmutableEntry<>(13.0, 0.25 - 0.140),
                    new AbstractMap.SimpleImmutableEntry<>(14.0, 0.25 - 0.139),
                    new AbstractMap.SimpleImmutableEntry<>(15.0, 0.25 - 0.138),
                    new AbstractMap.SimpleImmutableEntry<>(16.0, 0.25 - 0.137),
                    new AbstractMap.SimpleImmutableEntry<>(17.0, 0.25 - 0.1367),
                    new AbstractMap.SimpleImmutableEntry<>(18.0, 0.25 - 0.1361),
                    new AbstractMap.SimpleImmutableEntry<>(19.0, 0.25 - 0.1356),
                    new AbstractMap.SimpleImmutableEntry<>(20.0, 0.25 - 0.135)
            ),
            scoringDistanceToTOF = InterpolatingDoubleTreeMap.ofEntries(
                    new AbstractMap.SimpleImmutableEntry<>(1.0, 0.6932),
                    new AbstractMap.SimpleImmutableEntry<>(2.0, 0.8273),
                    new AbstractMap.SimpleImmutableEntry<>(3.0, 0.9425),
                    new AbstractMap.SimpleImmutableEntry<>(4.0, 1.045),
                    new AbstractMap.SimpleImmutableEntry<>(5.0, 1.1384),
                    new AbstractMap.SimpleImmutableEntry<>(6.0, 1.2247),
                    new AbstractMap.SimpleImmutableEntry<>(7.0, 1.3053),
                    new AbstractMap.SimpleImmutableEntry<>(8.0, 1.3812),
                    new AbstractMap.SimpleImmutableEntry<>(9.0, 1.4531),
                    new AbstractMap.SimpleImmutableEntry<>(10.0, 1.5216),
                    new AbstractMap.SimpleImmutableEntry<>(11.0, 1.5872),
                    new AbstractMap.SimpleImmutableEntry<>(12.0, 1.6502),
                    new AbstractMap.SimpleImmutableEntry<>(13.0, 1.7108),
                    new AbstractMap.SimpleImmutableEntry<>(14.0, 1.7694),
                    new AbstractMap.SimpleImmutableEntry<>(15.0, 1.8261),
                    new AbstractMap.SimpleImmutableEntry<>(16.0, 1.8811),
                    new AbstractMap.SimpleImmutableEntry<>(17.0, 1.9345),
                    new AbstractMap.SimpleImmutableEntry<>(18.0, 1.9865),
                    new AbstractMap.SimpleImmutableEntry<>(19.0, 2.0372),
                    new AbstractMap.SimpleImmutableEntry<>(20.0, 2.0866)
            );

    public static final ShooterLookupTable shooterLookupTable = new ShooterLookupTable(
            scoringDistanceToRPM,
            scoringDistanceToHoodAngle,
            scoringDistanceToTOF
    );
}
