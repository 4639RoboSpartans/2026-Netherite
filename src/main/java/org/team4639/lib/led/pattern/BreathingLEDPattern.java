/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern;

import edu.wpi.first.wpilibj.util.Color;

public final class BreathingLEDPattern implements LEDPattern {
    private final Color color;
    private final double speed;
    private final double percentage_min;
    private final double percentage_max;

    /**
     * @param color The color to use
     * @param speed The speed, in cycles per second, of the breathing effect
     */
    public BreathingLEDPattern(Color color, double speed) {
        this(color, speed, 0, 1);
    }

    /**
     * @param color The color to use
     * @param speed The speed, in cycles per second, of the breathing effect
     * @param minBrightness The minimum brightness to use
     * @param maxBrightness The maximum brightness to use
     */
    public BreathingLEDPattern(Color color, double speed, double minBrightness, double maxBrightness) {
        this.color = color;
        this.speed = speed;
        this.percentage_min = minBrightness;
        this.percentage_max = maxBrightness;
    }

    @Override
    public Color get(int led, double time) {
        double factor =
                percentage_min + (percentage_max - percentage_min) * (Math.sin(time * speed * Math.PI * 2) * 0.5 + 0.5);

        double red = color.red * factor;
        double green = color.green * factor;
        double blue = color.blue * factor;

        return new Color(red, green, blue);
    }
}
