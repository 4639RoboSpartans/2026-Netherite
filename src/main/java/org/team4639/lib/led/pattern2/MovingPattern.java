/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern2;

import edu.wpi.first.wpilibj.util.Color;
import org.team4639.lib.led.pattern.LEDPattern;

public class MovingPattern implements LEDPattern {
    private LEDPattern pattern;
    private double ledsPerSecond;

    public MovingPattern(LEDPattern pattern, double ledsPerSecond) {
        this.pattern = pattern;
        this.ledsPerSecond = ledsPerSecond;
    }

    @Override
    public Color get(int led, double time) {
        return pattern.get(led + (int) (ledsPerSecond * time), time);
    }
}
