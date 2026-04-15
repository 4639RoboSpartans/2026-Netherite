/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern;

import edu.wpi.first.wpilibj.util.Color;

public class RainbowLEDPattern extends BasicLEDPattern {
    public RainbowLEDPattern(int cycleSize, double speed) {
        super(1, speed, getRainbowPattern(cycleSize));
    }

    private static Color[] getRainbowPattern(int cycleSize) {
        Color[] colors = new Color[cycleSize];
        for (int i = 0; i < cycleSize; i++) {
            colors[i] = Color.fromHSV(180 * i / cycleSize, 255, 255);
        }
        return colors;
    }
}
