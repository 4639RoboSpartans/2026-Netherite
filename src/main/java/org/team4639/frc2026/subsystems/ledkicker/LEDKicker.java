/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.ledkicker;

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

  public void setPattern(LEDPattern pattern) {
    io.setPattern(pattern);
  }
}
