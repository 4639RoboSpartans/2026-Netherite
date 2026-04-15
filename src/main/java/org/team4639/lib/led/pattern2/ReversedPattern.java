/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern2;

import edu.wpi.first.wpilibj.util.Color;
import org.team4639.lib.led.pattern.LEDPattern;

public class ReversedPattern implements LEDPattern {
    private LEDPattern pattern;
    private int startingIndex;

    public ReversedPattern(LEDPattern pattern) {
        this.pattern = pattern;
        startingIndex = 0;
    }

    public ReversedPattern(LEDPattern pattern, int startingIndex) {
        this.pattern = pattern;
        this.startingIndex = startingIndex;
    }

    @Override
    public Color get(int led, double time) {
        return pattern.get(startingIndex - led, time);
    }
}
