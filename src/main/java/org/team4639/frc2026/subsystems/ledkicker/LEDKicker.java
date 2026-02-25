/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.ledkicker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.littletonrobotics.junction.Logger;
import org.team4639.lib.led.pattern.LEDPattern;
import org.team4639.lib.util.FullSubsystem;

public class LEDKicker extends FullSubsystem {
    private final LEDKickerIO io;
    private final LEDKickerIOInputsAutoLogged inputs;

    public LEDKicker(LEDKickerIO io) {
        this.io = io;
        this.inputs = new LEDKickerIOInputsAutoLogged();
    }

    @Override
    public void periodicBeforeScheduler() {
        io.updateInputs(inputs);
        Logger.processInputs("LEDKicker", inputs);
    }

    private void setPatternInternal(LEDPattern pattern) {
        io.setPattern(pattern);
    }

    public InstantCommand setPattern(LEDPattern pattern) {
        return new InstantCommand(() -> this.setPatternInternal(pattern), this);
    }
}
