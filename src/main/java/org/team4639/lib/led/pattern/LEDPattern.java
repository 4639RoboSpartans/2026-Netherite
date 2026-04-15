/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern;

import edu.wpi.first.wpilibj.util.Color;
import java.util.Comparator;
import java.util.TreeMap;

public interface LEDPattern {
    LEDPattern BLANK = (led, time) -> new Color(0, 0, 0);

    Color get(int led, double time);

    static TreeMap<Color, LEDPattern> cache = new TreeMap<>(Comparator.comparingDouble(Color::hashCode));

    static Color mixColors(Color a, Color b, double aWeight) {
        double bWeight = 1 - aWeight;
        return new Color(
                a.red * aWeight + b.red * bWeight,
                a.green * aWeight + b.green * bWeight,
                a.blue * aWeight + b.blue * bWeight);
    }

    static LEDPattern ofColor(Color color) {
        return cache.computeIfAbsent(color, SolidLEDPattern::new);
    }
}
