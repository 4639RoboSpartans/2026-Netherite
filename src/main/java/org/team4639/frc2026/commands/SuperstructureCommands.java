/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team4639.frc2026.FieldConstants;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.subsystems.hood.Hood;
import org.team4639.frc2026.subsystems.kicker.Kicker;
import org.team4639.frc2026.subsystems.shooter.Shooter;
import org.team4639.frc2026.subsystems.spindexer.Spindexer;
import org.team4639.frc2026.subsystems.turret.Turret;

public class SuperstructureCommands {
  public static SuperstructureState currentState = SuperstructureState.IDLE;

  private static boolean runSpindexerWhileWaiting = false;

  public enum SuperstructureState {
    IDLE,
    PASS,
    SCORE,
    WAIT
  }

  public static Command idle(
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      RobotState state) {
    return Commands.run(
        () -> {
          shooter.setWantedState(Shooter.WantedState.IDLE);
          hood.setWantedState(Hood.WantedState.IDLE);
          spindexer.setWantedState(Spindexer.WantedState.IDLE);
          kicker.setWantedState(Kicker.WantedState.IDLE);

          turret.setWantedState(Turret.WantedState.HUB_TRACK);

          currentState = SuperstructureState.IDLE;
        },
        shooter.dummy,
        hood.dummy,
        turret.dummy,
        spindexer.dummy,
        kicker.dummy);
  }

  public static Command requestScoring(
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      RobotState state) {
    return new SequentialCommandGroup(
        Commands.run(
                () -> {
                  shooter.setWantedState(Shooter.WantedState.SCORING);
                  hood.setWantedState(avoidTrench(Hood.WantedState.SCORING, state));
                  turret.setWantedState(Turret.WantedState.SCORING);

                  spindexer.setWantedState(
                      runSpindexerWhileWaiting
                          ? Spindexer.WantedState.SPIN
                          : Spindexer.WantedState.IDLE);
                  kicker.setWantedState(Kicker.WantedState.IDLE);

                  currentState = SuperstructureState.WAIT;
                },
                shooter.dummy,
                hood.dummy,
                turret.dummy,
                spindexer.dummy,
                kicker.dummy)
            .until(() -> shooter.getSetpointRPM() != 0 && shooter.aboveSetpoint()),
        Commands.run(
            () -> {
              shooter.setWantedState(Shooter.WantedState.SCORING);
              hood.setWantedState(avoidTrench(Hood.WantedState.SCORING, state));
              turret.setWantedState(Turret.WantedState.SCORING);

              if (turret.atSetpoint() && state.withinShooterRange()) {
                spindexer.setWantedState(Spindexer.WantedState.SPIN);
                kicker.setWantedState(Kicker.WantedState.KICK);
                currentState = SuperstructureState.SCORE;
              } else {
                spindexer.setWantedState(
                    runSpindexerWhileWaiting
                        ? Spindexer.WantedState.SPIN
                        : Spindexer.WantedState.IDLE);
                kicker.setWantedState(Kicker.WantedState.IDLE);
                currentState = SuperstructureState.WAIT;
              }
            },
            shooter.dummy,
            hood.dummy,
            turret.dummy,
            spindexer.dummy,
            kicker.dummy));
  }

  public static Command requestPassing(
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      RobotState state) {
    return new SequentialCommandGroup(
        Commands.run(
                () -> {
                  shooter.setWantedState(Shooter.WantedState.PASSING);
                  hood.setWantedState(avoidTrench(Hood.WantedState.PASSING, state));
                  turret.setWantedState(Turret.WantedState.PASSING);

                  spindexer.setWantedState(
                      runSpindexerWhileWaiting
                          ? Spindexer.WantedState.SPIN
                          : Spindexer.WantedState.IDLE);
                  kicker.setWantedState(Kicker.WantedState.IDLE);
                },
                shooter.dummy,
                hood.dummy,
                turret.dummy,
                spindexer.dummy,
                kicker.dummy)
            .until(() -> shooter.getSetpointRPM() != 0 && shooter.aboveSetpoint()),
        Commands.run(
            () -> {
              shooter.setWantedState(Shooter.WantedState.PASSING);
              hood.setWantedState(avoidTrench(Hood.WantedState.PASSING, state));
              turret.setWantedState(Turret.WantedState.PASSING);

              if (turret.atSetpoint() && !state.passingWillHitHub()) {
                spindexer.setWantedState(Spindexer.WantedState.SPIN);
                kicker.setWantedState(Kicker.WantedState.KICK);
                currentState = SuperstructureState.PASS;
              } else {
                spindexer.setWantedState(
                    runSpindexerWhileWaiting
                        ? Spindexer.WantedState.SPIN
                        : Spindexer.WantedState.IDLE);
                kicker.setWantedState(Kicker.WantedState.IDLE);
                currentState = SuperstructureState.WAIT;
              }
            },
            shooter.dummy,
            hood.dummy,
            turret.dummy,
            spindexer.dummy,
            kicker.dummy));
  }

  public static Command autoSpinupToShoot(
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      RobotState state) {
    return Commands.run(
        () -> {
          shooter.setWantedState(Shooter.WantedState.SCORING);
          // don't want to raise since might go under trench in auto
          hood.setWantedState(Hood.WantedState.IDLE);
          turret.setWantedState(Turret.WantedState.SCORING);

          spindexer.setWantedState(
              runSpindexerWhileWaiting ? Spindexer.WantedState.SPIN : Spindexer.WantedState.IDLE);
          kicker.setWantedState(Kicker.WantedState.IDLE);

          currentState = SuperstructureState.WAIT;
        },
        shooter.dummy,
        hood.dummy,
        turret.dummy,
        spindexer.dummy,
        kicker.dummy);
  }

  private static Hood.WantedState avoidTrench(Hood.WantedState desiredState, RobotState state) {
    ChassisSpeeds speeds = state.getChassisSpeeds();
    Pose2d currentRobotPose = state.getEstimatedPose();
    Pose2d nextRobotPose =
        currentRobotPose.plus(
            new Transform2d(
                speeds.vxMetersPerSecond * 0.8,
                speeds.vyMetersPerSecond * 0.8,
                Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * 0.8)));

    if (Math.signum(nextRobotPose.getX() - FieldConstants.LinesVertical.hubCenter)
            != Math.signum(currentRobotPose.getX() - FieldConstants.LinesVertical.hubCenter)
        || Math.signum(nextRobotPose.getX() - FieldConstants.LinesVertical.oppHubCenter)
            != Math.signum(currentRobotPose.getX() - FieldConstants.LinesVertical.oppHubCenter)) {
      return Hood.WantedState.IDLE;
    }

    return desiredState;
  }
}
