/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern2;

import edu.wpi.first.wpilibj.util.Color;
import org.team4639.lib.led.pattern.LEDPattern;

public class BlinkingPattern implements LEDPattern {
    private double timeOfEachPattern;
    private LEDPattern[] patterns;
    private double period;

    public BlinkingPattern(double timeOfEachPattern, LEDPattern... patterns) {
        if (patterns.length == 0) throw new IllegalArgumentException("patterns cannot be empty!");
        this.timeOfEachPattern = timeOfEachPattern;
        this.patterns = patterns;
        period = timeOfEachPattern * patterns.length;
    }

    @Override
    public Color get(int led, double time) {
        // get the index of this time in the overall period
        double periodicTime = time % period;
        // return the pattern that corresponds when the period is equally divided by the number of
        // patterns
        return patterns[(int) (periodicTime / timeOfEachPattern)].get(led, time);
    }
}
