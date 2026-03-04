/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import org.team4639.frc2026.FieldConstants;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.constants.shooter.ScoringState;
import org.team4639.frc2026.subsystems.hood.Hood;
import org.team4639.frc2026.subsystems.kicker.Kicker;
import org.team4639.frc2026.subsystems.shooter.Shooter;
import org.team4639.frc2026.subsystems.spindexer.Spindexer;
import org.team4639.frc2026.subsystems.turret.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class Superstructure extends SubsystemBase{

    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;
    private final Kicker kicker;
    private final Spindexer spindexer;

    private final RobotState state;

    private final Runnable resetSuperstructure;

    private boolean turretHasSeenTarget = false;

    public Superstructure(Turret turret, Hood hood, Shooter shooter, Kicker kicker, Spindexer spindexer, RobotState state){
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.kicker = kicker;
        this.spindexer = spindexer;
        this.state = state;

        resetSuperstructure = () -> {
        hood.setWantedState(Hood.WantedState.IDLE);
        turret.setWantedState(Turret.WantedState.IDLE);
        shooter.setWantedState(Shooter.WantedState.IDLE);
        spindexer.setWantedState(Spindexer.WantedState.IDLE);
        kicker.setWantedState(Kicker.WantedState.IDLE);
    };
    }

    public Command idle() {
        return this.run(() -> {
            shooter.setWantedState(Shooter.WantedState.IDLE);
            kicker.setWantedState(Kicker.WantedState.IDLE);
            spindexer.setWantedState(Spindexer.WantedState.IDLE);
            turret.setWantedState(Turret.WantedState.IDLE);

            hood.setWantedState(Hood.WantedState.IDLE);
        }).finallyDo(resetSuperstructure);
    }

    public Command trackHub() {
        return this.run(() -> {
            shooter.setWantedState(Shooter.WantedState.IDLE);
            kicker.setWantedState(Kicker.WantedState.IDLE);
            spindexer.setWantedState(Spindexer.WantedState.IDLE);

            turret.setWantedState(Turret.WantedState.HUB_TRACK);
        }).finallyDo(resetSuperstructure);
    }

    public boolean shouldShoot() {
        return state.getEstimatedPose().getX() < FieldConstants.LinesVertical.allianceZone;
    }

    public boolean shouldPass() {
        return state.getEstimatedPose().getX() > FieldConstants.LinesVertical.allianceZone;
    }

    public Command requestScoring() {
        return spinupToScore()
                .andThen(requestShooting())
                .finallyDo(resetSuperstructure);
    }

    public Command requestPassing() {
        return spinupToPass()
                .andThen(requestShootingToPass())
                .finallyDo(resetSuperstructure);
    }

    private Command spinupToScore() {
        return this.run(() -> {
            hood.setWantedState(Hood.WantedState.SCORING);
            shooter.setWantedState(Shooter.WantedState.SCORING);
            turret.setWantedState(Turret.WantedState.SCORING);

            spindexer.setWantedState(Spindexer.WantedState.IDLE);
            kicker.setWantedState(Kicker.WantedState.IDLE);
        }).until(() -> shooter.getSetpointRPM() > 0 && shooter.atSetpoint());
    }

    private Command requestShooting() {
        return this.run(() -> {

            hood.setWantedState(Hood.WantedState.SCORING);
            shooter.setWantedState(Shooter.WantedState.SCORING);
            turret.setWantedState(
                    Turret.WantedState.SCORING);

            // hood will never change and shooter is allowed to fluctuate, mainly we are concerned about turret

            if (turret.atSetpoint()){
                spindexer.setWantedState(Spindexer.WantedState.SPIN);
                kicker.setWantedState(Kicker.WantedState.KICK);
            } else {
                spindexer.setWantedState(Spindexer.WantedState.IDLE);
                kicker.setWantedState(Kicker.WantedState.IDLE);
            }
        });
    }

    private Command spinupToPass() {
        return this.run(() -> {

            hood.setWantedState(Hood.WantedState.PASSING);
            shooter.setWantedState(Shooter.WantedState.PASSING);
            turret.setWantedState(
                    Turret.WantedState.PASSING);

            spindexer.setWantedState(Spindexer.WantedState.IDLE);
            kicker.setWantedState(Kicker.WantedState.IDLE);
        }).until(() -> shooter.getSetpointRPM() > 0 && shooter.atSetpoint());
    }

    private Command requestShootingToPass() {
        return this.run(() -> {

            hood.setWantedState(Hood.WantedState.PASSING);
            shooter.setWantedState(Shooter.WantedState.PASSING);
            turret.setWantedState(
                    Turret.WantedState.PASSING);

            // hood will never change and shooter is allowed to fluctuate, mainly we are concerned about turret

            if (turret.atSetpoint()){
                spindexer.setWantedState(Spindexer.WantedState.SPIN);
                kicker.setWantedState(Kicker.WantedState.KICK);
            } else {
                spindexer.setWantedState(Spindexer.WantedState.IDLE);
                kicker.setWantedState(Kicker.WantedState.IDLE);
            }
        });
    }
}
