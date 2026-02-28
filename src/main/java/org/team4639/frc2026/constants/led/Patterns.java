/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import org.team4639.lib.led.pattern.LEDPattern;
import org.team4639.lib.led.pattern2.BlockPattern;
import org.team4639.lib.led.pattern2.MovingPattern;
import org.team4639.lib.led.pattern2.RGBPattern;

public class Patterns {
    private static final int MOVING_BLOCK_SIZE = 6;
    private static final int LEDS_PER_SECOND = 30;

    public static final LEDPattern DEFAULT = color(Color.kOrange);
    public static final LEDPattern SHOOTER_REQUESTED = color(Color.kBlue);
    public static final LEDPattern SHOOTER_REQUESTED_AND_INTAKE = createMovingPattern(SHOOTER_REQUESTED);
    public static final LEDPattern SHOOTING = color(Color.kGreen);
    public static final LEDPattern SHOOTING_AND_INTAKE = createMovingPattern(SHOOTING);
    public static final LEDPattern DEFAULT_INTAKE = createMovingPattern(DEFAULT);

    public static final LEDPattern PASSING = color(Color.kWhite);
    public static final LEDPattern PASSING_AND_INTAKE = createMovingPattern(PASSING);

    public static final LEDPattern MANUAL = color(Color.kViolet);

    private static LEDPattern createMovingPattern(LEDPattern pattern) {
        return new MovingPattern(
                new BlockPattern(
                        MOVING_BLOCK_SIZE,
                        pattern,
                        LEDPattern.BLANK
                ),
                LEDS_PER_SECOND
        );
    }

    private static LEDPattern color(Color color){
        return new RGBPattern(LEDPattern.ofColor(color), new Color8Bit(0, 63, 0), new Color8Bit(255, 0, 0), new Color8Bit(0, 0, 31));
    }
}
