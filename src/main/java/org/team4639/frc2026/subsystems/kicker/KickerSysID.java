package org.team4639.frc2026.subsystems.kicker;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public abstract class KickerSysID {
    @Getter
    private SysIdRoutine routine;

    public static class KickerSysIDCTRE extends KickerSysID{
        public KickerSysIDCTRE(Kicker kicker){
            super();
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(12.0/10),
                            Volts.of(7),
                            null,
                            state -> SignalLogger.writeString("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            kicker::setVoltage,
                            null, // Signal Logger handles it,
                            kicker
                    )
            );
        }
    }

    public static class KickerSysIDWPI extends KickerSysID{
        public KickerSysIDWPI(Kicker kicker, KickerIO.KickerIOInputs inputs){
            super();
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(12.0/10),
                            Volts.of(7),
                            null,
                            state -> Logger.recordOutput("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            kicker::setVoltage,
                            log -> {
                                log.motor("roller")
                                        .angularPosition(Rotations.of(inputs.motorPosition))
                                        .angularVelocity(Rotations.of(inputs.motorVelocity).per(Second))
                                        .voltage(Volts.of(inputs.motorVoltage));
                            },
                            kicker
                    )
            );
        }
    }
}