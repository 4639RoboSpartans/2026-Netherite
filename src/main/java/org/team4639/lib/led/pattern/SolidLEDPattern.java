/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern;

import edu.wpi.first.wpilibj.util.Color;

public final class SolidLEDPattern implements LEDPattern {
    private final Color color;

    public SolidLEDPattern(Color color) {
        this.color = color;
    }

    @Override
    public Color get(int led, double time) {
        return color;
    }
}
