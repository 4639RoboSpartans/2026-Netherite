/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern;

import edu.wpi.first.wpilibj.util.Color;
import java.util.function.BiFunction;

public class MovingWaveLEDPattern implements LEDPattern {
    private final BiFunction<Integer, Double, Double> waveEquation;
    private final Color color1;
    private final Color color2;

    public MovingWaveLEDPattern(Color color1, Color color2, double frequency) {
        this.color1 = color1;
        this.color2 = color2;
        waveEquation = (led, time) -> Math.sin((frequency * 2 * Math.PI * time) - led);
    }

    @Override
    public Color get(int led, double time) {
        return Color.lerpRGB(color1, color2, waveEquation.apply(led, time));
    }
}
