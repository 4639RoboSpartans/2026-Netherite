/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  default void setSetpointDegrees(double setpointDegrees) {}

  default void setVoltage(double volts) {}

  default void updateInputs(HoodIOInputs inputs) {}

  default void applyNewGains() {}

  default void setPosition(double positionDegrees) {}

  @AutoLog
  class HoodIOInputs {
    public boolean connected = true;
    public boolean pivotEncoderConnected = true;
    public double volts = 0.0;
    public double amps = 0.0;
    public double celsius = 0.0;
    public double degrees = 0.0;
    public double degreesPerSecond = 0.0;
  }
}
