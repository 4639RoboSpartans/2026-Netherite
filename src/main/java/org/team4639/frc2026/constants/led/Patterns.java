/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.led;

import edu.wpi.first.wpilibj.util.Color;
import org.team4639.lib.led.pattern.LEDPattern;
import org.team4639.lib.led.pattern2.BlockPattern;
import org.team4639.lib.led.pattern2.MovingPattern;

public class Patterns {
    public static final LEDPattern DEFAULT = LEDPattern.ofColor(Color.kOrange);
    public static final LEDPattern SHOOTER_REQUESTED = LEDPattern.ofColor(Color.kBlue);
    public static final int MOVING_BLOCK_SIZE = 6;
    public static final int LEDS_PER_SECOND = 30;
    public static final LEDPattern SHOOTER_REQUESTED_AND_INTAKE = new MovingPattern(
            new BlockPattern(
                    MOVING_BLOCK_SIZE,
                    SHOOTER_REQUESTED,
                    LEDPattern.BLANK
            ),
            LEDS_PER_SECOND
    );
    public static final LEDPattern SHOOTING = LEDPattern.ofColor(Color.kGreen);
    public static final LEDPattern SHOOTING_AND_INTAKE = new MovingPattern(
            new BlockPattern(
                    MOVING_BLOCK_SIZE,
                    SHOOTING,
                    LEDPattern.BLANK
            ),
            LEDS_PER_SECOND
    );
    public static final LEDPattern DEFAULT_INTAKE = new MovingPattern(
            new BlockPattern(
                    MOVING_BLOCK_SIZE,
                    DEFAULT,
                    LEDPattern.BLANK
            ),
            LEDS_PER_SECOND
    );

    public static final LEDPattern PASSING = LEDPattern.ofColor(Color.kWhite);
    public static final LEDPattern PASSING_AND_INTAKE = new MovingPattern(
            new BlockPattern(
                    MOVING_BLOCK_SIZE,
                    PASSING,
                    LEDPattern.BLANK
            ),
            LEDS_PER_SECOND
    );

    public static final LEDPattern MANUAL = LEDPattern.ofColor(Color.kViolet);
}
