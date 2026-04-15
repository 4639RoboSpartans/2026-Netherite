/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern;

import edu.wpi.first.wpilibj.util.Color;

public class BasicLEDPattern implements LEDPattern {
    public final Color[] colors;
    private final int runLength;
    private final double speed;

    public BasicLEDPattern(int runLength, Color... colors) {
        this(runLength, 0, colors);
    }

    public BasicLEDPattern(int runLength, double speed, Color... colors) {
        this.runLength = runLength;
        this.speed = speed;
        this.colors = colors;
    }

    @Override
    public Color get(int led, double time) {
        int index = (int) Math.floor(led + time * speed);

        int colorIndex = ((index / runLength) % colors.length + colors.length) % colors.length;
        return colors[(colorIndex % colors.length + colors.length) % colors.length];
    }
}
