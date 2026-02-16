/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.constants.shooter.ScoringState;
import org.team4639.frc2026.subsystems.hood.Hood;
import org.team4639.frc2026.subsystems.shooter.Shooter;
import org.team4639.frc2026.subsystems.turret.Turret;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;

public class Superstructure extends SubsystemBase {
    private final RobotState state;
    private final Shooter shooter;
    private final Turret turret;
    private final Hood hood;
    public final Trigger scoringAtSetpointTrigger;

    public Superstructure(Shooter shooter, Turret turret, Hood hood, RobotState state) {
        this.state = state;
        this.shooter = shooter;
        this.turret = turret;
        this.hood = hood;
        scoringAtSetpointTrigger = new Trigger(this::scoringAtSetpoint);
    }

    @Override
    public void periodic() {
        ScoringState scoringState = state.calculateScoringState();
        shooter.setWantedState(Shooter.WantedState.SCORING, scoringState.shooterRPM().in(Rotations.per(Minute)));
        turret.setWantedState(Turret.WantedState.SCORING, scoringState.turretAngle().in(Rotations), 0);
        hood.setWantedState(Hood.WantedState.SCORING, scoringState.hoodAngle().in(Rotations));
    }

    public boolean scoringAtSetpoint() {
        return shooter.atSetpoint() && hood.atSetpoint() && turret.atSetpoint();
    }
}
