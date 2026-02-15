package org.team4639.frc2026.subsystems.spindexer;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public sealed class SpindexerSysID {
    @Getter
    private SysIdRoutine routine;

    public static final class SpindexerSysIDCTRE extends SpindexerSysID {
        public SpindexerSysIDCTRE(Spindexer spindexer){
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.5),
                            Volts.of(5),
                            null,
                            state -> SignalLogger.writeString("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            spindexer::setVoltage,
                            null, // SignalLogger handles logging
                            spindexer
                    )
            );
        }
    }

    public static final class SpindexerSysIDWPI extends SpindexerSysID {
        public SpindexerSysIDWPI(Spindexer spindexer, SpindexerIO.SpindexerIOInputs inputs){
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.5),
                            Volts.of(5),
                            null,
                            state -> Logger.recordOutput("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            spindexer::setVoltage,
                            log -> {
                                log.motor("spindexer")
                                        .angularPosition(Rotations.of(inputs.motorPosition))
                                        .angularVelocity(Rotations.of(inputs.motorVelocity).per(Second))
                                        .voltage(Volts.of(inputs.motorVoltage));
                            }, // SignalLogger handles logging
                            spindexer
                    )
            );
        }
    }
}
