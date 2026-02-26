/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.led;

import edu.wpi.first.wpilibj.util.Color;
import org.team4639.lib.led.pattern.LEDPattern;
import org.team4639.lib.led.pattern2.BlockPattern;
import org.team4639.lib.led.pattern2.MovingPattern;

public class Patterns {
    public static final LEDPattern DEFAULT = LEDPattern.ofColor(Color.kOrange);
    public static final LEDPattern SHOOTER_REQUESTED = LEDPattern.ofColor(Color.kBlue);
    public static final LEDPattern SHOOTER_REQUESTED_AND_INTAKE = new MovingPattern(
            new BlockPattern(
                    20,
                    SHOOTER_REQUESTED,
                    LEDPattern.BLANK
            ),
            10
    );
    public static final LEDPattern SHOOTING = LEDPattern.ofColor(Color.kGreen);
    public static final LEDPattern SHOOTING_AND_INTAKE = new MovingPattern(
            new BlockPattern(
                    20,
                    SHOOTING,
                    LEDPattern.BLANK
            ),
            10
    );
    public static final LEDPattern DEFAULT_INTAKE = new MovingPattern(
            new BlockPattern(
                    20,
                    DEFAULT,
                    LEDPattern.BLANK
            ),
            10
    );

    public static final LEDPattern PASSING = LEDPattern.ofColor(Color.kWhite);
    public static final LEDPattern PASSING_AND_INTAKE = new MovingPattern(
            new BlockPattern(
                    20,
                    PASSING,
                    LEDPattern.BLANK
            ),
            10
    );

    public static final LEDPattern MANUAL = LEDPattern.ofColor(Color.kViolet);
}
