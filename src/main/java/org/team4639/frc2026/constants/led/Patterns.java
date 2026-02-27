/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.led;

import edu.wpi.first.wpilibj.util.Color;
import org.team4639.lib.led.pattern.LEDPattern;
import org.team4639.lib.led.pattern2.BlockPattern;
import org.team4639.lib.led.pattern2.MovingPattern;
import org.team4639.lib.led.pattern2.RGBPattern;

public class Patterns {
    private static final int MOVING_BLOCK_SIZE = 6;
    private static final int LEDS_PER_SECOND = 30;

    public static final LEDPattern DEFAULT = RGBPattern.GRB(Color.kOrange);
    public static final LEDPattern SHOOTER_REQUESTED = RGBPattern.GRB(Color.kBlue);
    public static final LEDPattern SHOOTER_REQUESTED_AND_INTAKE = createMovingPattern(SHOOTER_REQUESTED);
    public static final LEDPattern SHOOTING = RGBPattern.GRB(Color.kGreen);
    public static final LEDPattern SHOOTING_AND_INTAKE = createMovingPattern(SHOOTING);
    public static final LEDPattern DEFAULT_INTAKE = createMovingPattern(DEFAULT);

    public static final LEDPattern PASSING = RGBPattern.GRB(Color.kWhite);
    public static final LEDPattern PASSING_AND_INTAKE = createMovingPattern(PASSING);

    public static final LEDPattern MANUAL = RGBPattern.GRB(Color.kViolet);

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
}
