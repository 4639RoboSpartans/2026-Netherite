/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public abstract sealed class TurretSysID {
  private final Turret turret;
  private final TurretIO.TurretIOInputs inputs;

  @Getter private SysIdRoutine routine;

  public static final class TurretSysIDWPI extends TurretSysID {
    public TurretSysIDWPI(Turret turret, TurretIO.TurretIOInputs inputs) {
      super(turret, inputs);
      super.routine =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  Volts.per(Second).of(0.5),
                  Volts.of(6),
                  Seconds.of(16),
                  (state) -> SignalLogger.writeString("Turret SysID State", state.toString())),
              new SysIdRoutine.Mechanism(turret::setVoltage, log -> {}, turret));
    }
  }
}
