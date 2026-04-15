/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern2;

import edu.wpi.first.wpilibj.util.Color;
import org.team4639.lib.led.pattern.LEDPattern;

public class BlockPattern implements LEDPattern {
    private int blockSize;
    private LEDPattern[] patterns;
    private int ledPeriod;

    public BlockPattern(int blockSizeLEDs, LEDPattern... patterns) {
        if (patterns.length == 0) throw new IllegalArgumentException("patterns cannot be empty!");
        blockSize = blockSizeLEDs;
        this.patterns = patterns;
        ledPeriod = blockSize * patterns.length;
    }

    @Override
    public Color get(int led, double time) {
        // get the index of this LED in the overall period
        int periodicLED = led % ledPeriod;
        // return the pattern that corresponds when the period is equally divided by the number of
        // blocks
        return patterns[periodicLED / blockSize].get(led, time);
    }
}
