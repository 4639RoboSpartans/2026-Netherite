/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led.subsystem;

import java.util.Objects;
import org.team4639.lib.led.pattern.LEDPattern;

public class DummyLEDStrip extends LEDStrip {

  private static volatile LEDStrip instance;

  static synchronized LEDStrip getInstance() {
    return Objects.requireNonNullElseGet(instance, () -> instance = new DummyLEDStrip());
  }

  @Override
  public void setPattern(LEDPattern pattern) {}

  @Override
  public void update() {}
}
