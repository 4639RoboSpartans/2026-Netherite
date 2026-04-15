/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.team4639.lib.led.pattern.LEDPattern;

public abstract class LEDStrip extends SubsystemBase {
    public abstract void setPattern(LEDPattern pattern);

    public abstract void update();

    @Override
    public void periodic() {
        update();
    }

    public void resetToBlank() {
        setPattern(LEDPattern.BLANK);
    }

    public Command usePattern(LEDPattern pattern) {
        return usePattern(() -> pattern);
    }

    public Command usePattern(Supplier<LEDPattern> patternSupplier) {
        return run(() -> this.setPattern(patternSupplier.get()))
                .finallyDo(() -> resetToBlank())
                .ignoringDisable(true);
    }
}
