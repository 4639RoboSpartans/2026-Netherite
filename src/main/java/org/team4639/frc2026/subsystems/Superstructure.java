/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.subsystems.hood.Hood;
import org.team4639.frc2026.subsystems.kicker.Kicker;
import org.team4639.frc2026.subsystems.shooter.Shooter;
import org.team4639.frc2026.subsystems.spindexer.Spindexer;
import org.team4639.frc2026.subsystems.turret.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Superstructure extends SubsystemBase{

    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final Kicker kicker;
    private final Spindexer spindexer;

    private final RobotState state;

    public Command holdTurretAt180() {
        return this.run(() -> {
            turret.setWantedState(Turret.WantedState.SCORING, 0.5 - state.getEstimatedPose().getRotation().getRotations(), 0);
        }).finallyDo(() -> turret.setWantedState(Turret.WantedState.IDLE));
    }

    public Command hoodUp() {
        return this.run(() -> {
            hood.setWantedState(Hood.WantedState.SCORING, Degrees.of(50).in(Rotations));
        }).finallyDo(() -> hood.setWantedState(Hood.WantedState.IDLE));
    }

    public Command hoodDown() {
        return this.run(() -> {
            hood.setWantedState(Hood.WantedState.SCORING, Degrees.of(20).in(Rotations));
        }).finallyDo(() -> hood.setWantedState(Hood.WantedState.IDLE));
    }
}
