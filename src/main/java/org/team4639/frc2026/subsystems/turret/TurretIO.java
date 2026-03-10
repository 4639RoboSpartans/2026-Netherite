/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  default void setRotorRotationSetpoint(double rotation) {}

  default void setRotorRotationSetpoint(double rotation, double velocityrps) {}

  default void setVoltage(double voltage) {}

  default void updateInputs(TurretIOInputs inputs) {}

  default void applyNewGains() {}

  @AutoLog
  class TurretIOInputs {
    public boolean connected = true;
    public double volts = 0.0;
    public double amps = 0.0;
    public double celsius = 0.0;
    public double rotationsPerSecond = 0.0;
    public double rotations = 0.0;
  }
}
