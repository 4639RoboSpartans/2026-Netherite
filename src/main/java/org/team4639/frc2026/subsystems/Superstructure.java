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

            // if we have no vision targets, flip the turret out

            if (state.getEstimatedPose().equals(Pose2d.kZero)) {
                turret.setWantedState(
                        Turret.WantedState.SCORING,
                        0.5 - state.getEstimatedPose().getRotation().getRotations(),
                        0);
            } else {
                turret.setWantedState(
                        Turret.WantedState.SCORING,
                        state.calculateScoringState().turretAngle().in(Rotations) - state.getEstimatedPose().getRotation().getRotations(),
                        0);
            }

            // decide whether to put the hood in or out
            var speeds = state.getChassisSpeeds();
            // our current speed would take us through the trench in 0.5s, or we are already in our alliance zone
            if (
                    state.getEstimatedPose()
                            .plus(new Transform2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, Rotation2d.kZero).times(0.5))
                            .getX()
                            < FieldConstants.LinesVertical.allianceZone
                                    + FieldConstants.RightBump.depth
                || state.getEstimatedPose().getX() < FieldConstants.LinesVertical.allianceZone + FieldConstants.RightBump.depth)
            {
                hood.setWantedState(Hood.WantedState.SCORING, Units.degreesToRotations(20));
            } else {
                hood.setWantedState(Hood.WantedState.SCORING, Units.degreesToRotations(50));
            }
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

    private Command spinupToScore() {
        return this.run(() -> {
            var scoringState = state.calculateScoringState();

            hood.setWantedState(Hood.WantedState.SCORING, scoringState.hoodAngle().in(Rotations));
            shooter.setWantedState(Shooter.WantedState.SCORING, scoringState.shooterRPM().in(Rotations.per(Minute)));
            turret.setWantedState(
                    Turret.WantedState.SCORING,
                    scoringState.turretAngle().in(Rotations) - state.getEstimatedPose().getRotation().getRotations(),
                    0);

            spindexer.setWantedState(Spindexer.WantedState.IDLE);
            kicker.setWantedState(Kicker.WantedState.IDLE);
        }).until(() -> shooter.getSetpointRPM() > 0 && shooter.atSetpoint());
    }

    private Command requestShooting() {
        return this.run(() -> {
            var scoringState = state.calculateScoringState();

            hood.setWantedState(Hood.WantedState.SCORING, scoringState.hoodAngle().in(Rotations));
            shooter.setWantedState(Shooter.WantedState.SCORING, scoringState.shooterRPM().in(Rotations.per(Minute)));
            turret.setWantedState(
                    Turret.WantedState.SCORING,
                    scoringState.turretAngle().in(Rotations) - state.getEstimatedPose().getRotation().getRotations(),
                    0);

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
            var scoringState = state.calculatePassingState();

            hood.setWantedState(Hood.WantedState.PASSING, scoringState.hoodAngle().in(Rotations));
            shooter.setWantedState(Shooter.WantedState.PASSING, scoringState.shooterRPM().in(Rotations.per(Minute)));
            turret.setWantedState(
                    Turret.WantedState.PASSING,
                    0,
                    scoringState.turretAngle().in(Rotations) - state.getEstimatedPose().getRotation().getRotations());

            spindexer.setWantedState(Spindexer.WantedState.IDLE);
            kicker.setWantedState(Kicker.WantedState.IDLE);
        }).until(() -> shooter.getSetpointRPM() > 0 && shooter.atSetpoint());
    }

    private Command requestShootingToPass() {
        return this.run(() -> {
            var scoringState = state.calculatePassingState();

            hood.setWantedState(Hood.WantedState.PASSING, scoringState.hoodAngle().in(Rotations));
            shooter.setWantedState(Shooter.WantedState.PASSING, scoringState.shooterRPM().in(Rotations.per(Minute)));
            turret.setWantedState(
                    Turret.WantedState.PASSING,
                    0,
                    scoringState.turretAngle().in(Rotations) - state.getEstimatedPose().getRotation().getRotations());

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

    // actions for testing

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

    public Command shootDemo() {
        var scoringState = new ScoringState(Rotations.per(Minute).of(1000), Degrees.of(20), Degrees.of(180));
        return this.run(() -> {

            hood.setWantedState(Hood.WantedState.SCORING, scoringState.hoodAngle().in(Rotations));
            shooter.setWantedState(Shooter.WantedState.SCORING, scoringState.shooterRPM().in(Rotations.per(Minute)));
            turret.setWantedState(
                    Turret.WantedState.SCORING,
                    scoringState.turretAngle().in(Rotations) - state.getEstimatedPose().getRotation().getRotations(),
                    0);

            spindexer.setWantedState(Spindexer.WantedState.IDLE);
            kicker.setWantedState(Kicker.WantedState.IDLE);
        }).until(() -> shooter.getSetpointRPM() > 0 && shooter.atSetpoint())
                .andThen(
                        this.run(() -> {

                            hood.setWantedState(Hood.WantedState.SCORING, scoringState.hoodAngle().in(Rotations));
                            shooter.setWantedState(Shooter.WantedState.SCORING, scoringState.shooterRPM().in(Rotations.per(Minute)));
                            turret.setWantedState(
                                    Turret.WantedState.SCORING,
                                    scoringState.turretAngle().in(Rotations) - state.getEstimatedPose().getRotation().getRotations(),
                                    0);

                            // hood will never change and shooter is allowed to fluctuate, mainly we are concerned about turret

                            if (turret.atSetpoint()){
                                spindexer.setWantedState(Spindexer.WantedState.SPIN);
                                kicker.setWantedState(Kicker.WantedState.KICK);
                            } else {
                                spindexer.setWantedState(Spindexer.WantedState.IDLE);
                                kicker.setWantedState(Kicker.WantedState.IDLE);
                            }
                        })
                ).finallyDo(resetSuperstructure);
    }

    public Command passDemo() {
        var scoringState = new ScoringState(Rotations.per(Minute).of(1000), Degrees.of(50), Degrees.of(180));
        return this.run(() -> {

            hood.setWantedState(Hood.WantedState.PASSING, scoringState.hoodAngle().in(Rotations));
            shooter.setWantedState(Shooter.WantedState.PASSING, scoringState.shooterRPM().in(Rotations.per(Minute)));
            turret.setWantedState(
                    Turret.WantedState.PASSING,
                    0,
                    scoringState.turretAngle().in(Rotations) - state.getEstimatedPose().getRotation().getRotations());

            spindexer.setWantedState(Spindexer.WantedState.IDLE);
            kicker.setWantedState(Kicker.WantedState.IDLE);
        }).until(() -> shooter.getSetpointRPM() > 0 && shooter.atSetpoint())
                .andThen(
                        this.run(() -> {

                            hood.setWantedState(Hood.WantedState.PASSING, scoringState.hoodAngle().in(Rotations));
                            shooter.setWantedState(Shooter.WantedState.PASSING, scoringState.shooterRPM().in(Rotations.per(Minute)));
                            turret.setWantedState(
                                    Turret.WantedState.PASSING,
                                    0,
                                    scoringState.turretAngle().in(Rotations) - state.getEstimatedPose().getRotation().getRotations());

                            // hood will never change and shooter is allowed to fluctuate, mainly we are concerned about turret

                            if (turret.atSetpoint()){
                                spindexer.setWantedState(Spindexer.WantedState.SPIN);
                                kicker.setWantedState(Kicker.WantedState.KICK);
                            } else {
                                spindexer.setWantedState(Spindexer.WantedState.IDLE);
                                kicker.setWantedState(Kicker.WantedState.IDLE);
                            }
                        })
                ).finallyDo(resetSuperstructure);
    }
}
