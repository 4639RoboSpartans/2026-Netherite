/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.pattern;

import edu.wpi.first.wpilibj.util.Color;
import java.util.function.BiFunction;

public class MovingLEDPattern implements LEDPattern {
    private final BiFunction<Integer, Double, Double> waveEquation;
    private final Color color1;
    private final Color color2;
    private final Double brightness;

    public MovingLEDPattern(Color color1, Color color2, double frequency, double scale, double brightness) {
        this.color1 = color1;
        this.color2 = color2;
        waveEquation = (led, time) -> Math.sin((frequency * 2 * Math.PI * time) - led * scale);
        this.brightness = brightness;
    }

    @Override
    public Color get(int led, double time) {
        return Color.lerpRGB(Color.kBlack, (waveEquation.apply(led, time) > 0 ? color1 : color2), brightness);
    }
}
