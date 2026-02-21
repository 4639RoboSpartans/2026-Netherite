/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.subsystems.hood.HoodIO.HoodIOInputs;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public sealed abstract class HoodSysID {
    @Getter
    private SysIdRoutine routine;
    private final Hood hood;
    private final HoodIOInputs inputs;

    public static final class HoodSysIDWPI extends HoodSysID{
        public HoodSysIDWPI(Hood hood, HoodIOInputs inputs){
            super(hood, inputs);
            super.routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.per(Second).of(0.1),
                    Volts.of(0.5), null,
                    (state) -> Logger.recordOutput("SysIdTestState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    hood::setVoltage,
                    log -> {
                        log.motor("hood")
                        .angularVelocity(Degrees.per(Second).of(inputs.pivotVelocityDegrees))
                        .angularPosition(Degrees.of(inputs.pivotPositionDegrees))
                        .voltage(Volts.of(inputs.pivotVoltage));
                     },
                    hood
                )
            );
            }
    }

    public static final class HoodSysIDCTRE extends HoodSysID{
        public HoodSysIDCTRE(Hood hood, HoodIOInputs inputs){
            super(hood, inputs);
            super.routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.per(Second).of(0.1),
                    Volts.of(0.5), null,
                    (state) -> SignalLogger.writeString("SysIDTestState", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    hood::setVoltage,
                    null, // SignalLogger
                    hood
                )
            );
            }
    }
}
