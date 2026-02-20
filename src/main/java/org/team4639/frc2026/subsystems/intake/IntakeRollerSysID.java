/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intake;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public abstract class IntakeRollerSysID {
    @Getter
    private SysIdRoutine routine;

    public static class IntakeRollerSysIDCTRE extends IntakeRollerSysID{
        public IntakeRollerSysIDCTRE(Intake intake){
            super();
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(6.0/10),
                            Volts.of(7),
                            null,
                            state -> SignalLogger.writeString("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            voltage -> intake.setRollerVoltage(voltage.in(Volts)),
                            null, // Signal Logger handles it,
                            intake
                    )
            );
        }
    }

    public static class IntakeRollerSysIDWPI extends IntakeRollerSysID{
        public IntakeRollerSysIDWPI(Intake intake, IntakeRollerIO.IntakeRollerIOInputs inputs){
            super();
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(6.0/10),
                            Volts.of(7),
                            null,
                            state -> Logger.recordOutput("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            voltage -> intake.setRollerVoltage(voltage.in(Volts)),
                            log -> {
                                log.motor("roller")
                                        .angularPosition(Rotations.of(inputs.position))
                                        .angularVelocity(Rotations.of(inputs.velocity).per(Second))
                                        .voltage(Volts.of(inputs.voltage));
                            },
                            intake
                    )
            );
        }
    }
}
