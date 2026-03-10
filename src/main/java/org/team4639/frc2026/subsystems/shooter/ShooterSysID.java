/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.subsystems.shooter.ShooterIO.ShooterIOInputs;
import org.team4639.lib.unit.Units2;

@RequiredArgsConstructor
public abstract sealed class ShooterSysID {
  private final Shooter shooter;
  private final ShooterIOInputs inputs;

  @Getter private SysIdRoutine routine;

  public static final class ShooterSysIDWPI extends ShooterSysID {
    public ShooterSysIDWPI(Shooter shooter, ShooterIOInputs inputs) {
      super(shooter, inputs);
      super.routine =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  Volts.per(Second).of(0.25),
                  Volts.of(3),
                  null,
                  (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
              new SysIdRoutine.Mechanism(
                  shooter::setVoltage,
                  log -> {
                    // default = REV, left is leader
                    log.motor("Shooter")
                        .angularVelocity(Units2.RPM.of(inputs.leftRPM))
                        .angularPosition(Rotations.of(inputs.leftRotations))
                        .voltage(Volts.of(inputs.leftVolts));
                  },
                  shooter));
    }
  }

  public static final class ShooterSysIDURCL extends ShooterSysID {
    public ShooterSysIDURCL(Shooter shooter, ShooterIOInputs inputs) {
      super(shooter, inputs);
      super.routine =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  Volts.per(Second).of(0.25),
                  Volts.of(3),
                  null,
                  (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
              new SysIdRoutine.Mechanism(
                  shooter::setVoltage,
                  null // record URCL data, left motor should be used as leader for SparkFlex io
                  ,
                  shooter));
    }
  }

  public static final class ShooterSysIDCTRE extends ShooterSysID {
    public ShooterSysIDCTRE(Shooter shooter, ShooterIOInputs inputs) {
      super(shooter, inputs);
      super.routine =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  Volts.per(Second).of(0.25),
                  Volts.of(3),
                  null,
                  (state) -> SignalLogger.writeString("SysIdTestState", state.toString())),
              new SysIdRoutine.Mechanism(
                  shooter::setVoltage,
                  null // record SignalLogger data, right motor should be used as leader in io
                  // talonFX
                  ,
                  shooter));
    }
  }
}
