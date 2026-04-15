/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.ledkicker;

import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.led.pattern.LEDPattern;
import org.team4639.lib.led.subsystem.LEDStrip;
import org.team4639.lib.led.subsystem.PhysicalLEDStrip;

public class LEDKickerIOHardware extends LEDKickerIO {
    private final LEDStrip ledStrip;

    public LEDKickerIOHardware(PortConfiguration portConfiguration, Integer length) {
        this.ledStrip = new PhysicalLEDStrip(portConfiguration.ledkicker, length);
    }

    @Override
    public void setPattern(LEDPattern pattern) {
        ledStrip.setPattern(pattern);
    }

    @Override
    public void updateInputs(LEDKickerIOInputs inputs) {}
}
