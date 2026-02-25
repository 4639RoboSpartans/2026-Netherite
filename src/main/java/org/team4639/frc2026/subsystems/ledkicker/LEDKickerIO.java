/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.ledkicker;

import org.littletonrobotics.junction.AutoLog;
import org.team4639.lib.led.pattern.LEDPattern;

public abstract class LEDKickerIO {
    @AutoLog
    public static class LEDKickerIOInputs{

    }

    public void setPattern(LEDPattern pattern) {}

    public void updateInputs(LEDKickerIOInputs inputs) {}
}
