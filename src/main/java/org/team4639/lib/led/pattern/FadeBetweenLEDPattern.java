/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern;

import edu.wpi.first.wpilibj.util.Color;

public class FadeBetweenLEDPattern implements LEDPattern {
    private final double speed;
    private final Color[] colors;

    /**
     * @param speed The speed of blinking, in transitions per second. The period of the entire cycle
     *     will be (len(colors) - 1) / speed
     * @param colors The list of colors to cycle between
     */
    public FadeBetweenLEDPattern(double speed, Color... colors) {
        this.speed = speed;
        this.colors = colors;
    }

    @Override
    public Color get(int led, double time) {
        double progress = time * speed;

        int currColorIndex = (int) progress % (colors.length - 1);
        int nextColorIndex = currColorIndex + 1;

        return LEDPattern.mixColors(colors[currColorIndex], colors[nextColorIndex], progress % 1);
    }
}
