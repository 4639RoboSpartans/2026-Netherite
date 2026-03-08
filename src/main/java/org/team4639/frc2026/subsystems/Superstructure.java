/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import org.team4639.frc2026.Constants;
import org.team4639.frc2026.FieldConstants;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.constants.shooter.ShooterScoringData;
import org.team4639.frc2026.subsystems.hood.Hood;
import org.team4639.frc2026.subsystems.kicker.Kicker;
import org.team4639.frc2026.subsystems.shooter.Shooter;
import org.team4639.frc2026.subsystems.spindexer.Spindexer;
import org.team4639.frc2026.subsystems.turret.Turret;

public class Superstructure extends SubsystemBase {

  private final Turret turret;
  private final Hood hood;
  private final Shooter shooter;
  private final Kicker kicker;
  private final Spindexer spindexer;

  private final RobotState state;

  private final Runnable resetSuperstructure;

  private boolean spinUpShooterWhileIdle = false;
  private double SPIN_UP_TIME_PERIOD = 0.5;

  private boolean disableTurret = false;

  public Superstructure(
      Turret turret,
      Hood hood,
      Shooter shooter,
      Kicker kicker,
      Spindexer spindexer,
      RobotState state) {
    this.turret = turret;
    this.hood = hood;
    this.shooter = shooter;
    this.kicker = kicker;
    this.spindexer = spindexer;
    this.state = state;

    resetSuperstructure =
        () -> {
          hood.setWantedState(Hood.WantedState.IDLE);
          turret.setWantedState(Turret.WantedState.IDLE);
          shooter.setWantedState(Shooter.WantedState.IDLE);
          spindexer.setWantedState(Spindexer.WantedState.IDLE);
          kicker.setWantedState(Kicker.WantedState.IDLE);
        };
  }

  public Command idle() {
    return this.run(
            () -> {
              if (spinUpShooterWhileIdle)
                shooter.setWantedState(
                    state.robotInAllianceZone.getAsBoolean()
                        ? Shooter.WantedState.SCORING
                        : Shooter.WantedState.PASSING);
              else shooter.setWantedState(Shooter.WantedState.IDLE);

              kicker.setWantedState(Kicker.WantedState.IDLE);
              spindexer.setWantedState(Spindexer.WantedState.IDLE);
              turret.setWantedState(
                  disableTurret ? Turret.WantedState.IDLE : Turret.WantedState.HUB_TRACK);

              hood.setWantedState(Hood.WantedState.IDLE);

              state.setSuperstructureState(
                  disableTurret
                      ? GenericSuperstructureState.MANUAL
                      : GenericSuperstructureState.IDLE);
            })
        .finallyDo(resetSuperstructure);
  }

  public Command trackHub() {
    return this.run(
            () -> {
              if (spinUpShooterWhileIdle)
                shooter.setWantedState(
                    state.robotInAllianceZone.getAsBoolean()
                        ? Shooter.WantedState.SCORING
                        : Shooter.WantedState.PASSING);
              else shooter.setWantedState(Shooter.WantedState.IDLE);

              kicker.setWantedState(Kicker.WantedState.IDLE);
              spindexer.setWantedState(Spindexer.WantedState.IDLE);

              turret.setWantedState(Turret.WantedState.HUB_TRACK);

              state.setSuperstructureState(GenericSuperstructureState.IDLE);
            })
        .finallyDo(resetSuperstructure);
  }

  public boolean shouldShoot() {
    return state.getEstimatedPose().getX() < FieldConstants.LinesVertical.allianceZone;
  }

  public boolean shouldPass() {
    return state.getEstimatedPose().getX() > FieldConstants.LinesVertical.allianceZone;
  }

  public Command requestScoring() {
    return spinupToScore().andThen(requestShooting()).finallyDo(resetSuperstructure);
  }

  public Command requestPassing() {
    return spinupToPass().andThen(requestShootingToPass()).finallyDo(resetSuperstructure);
  }

  private Command spinupToScore() {
    return this.run(
            () -> {
              hood.setWantedState(Hood.WantedState.SCORING);
              shooter.setWantedState(Shooter.WantedState.SCORING);
              turret.setWantedState(Turret.WantedState.SCORING);

              spindexer.setWantedState(Spindexer.WantedState.IDLE);
              kicker.setWantedState(Kicker.WantedState.IDLE);

              state.setSuperstructureState(GenericSuperstructureState.WAITING);
            })
        .until(() -> shooter.getSetpointRPM() > 0 && shooter.atSetpoint());
  }

  private Command requestShooting() {
    return this.run(
        () -> {
          hood.setWantedState(Hood.WantedState.SCORING);
          shooter.setWantedState(Shooter.WantedState.SCORING);
          turret.setWantedState(Turret.WantedState.SCORING);

          // hood will never change and shooter is allowed to fluctuate, mainly we are concerned
          // about turret

          if (turret.atSetpoint()) {
            spindexer.setWantedState(Spindexer.WantedState.SPIN);
            kicker.setWantedState(Kicker.WantedState.KICK);

            state.setSuperstructureState(GenericSuperstructureState.SHOOT);
          } else {
            spindexer.setWantedState(Spindexer.WantedState.IDLE);
            kicker.setWantedState(Kicker.WantedState.IDLE);

            state.setSuperstructureState(GenericSuperstructureState.WAITING);
          }
        });
  }

  private Command spinupToPass() {
    return this.run(
            () -> {
              hood.setWantedState(Hood.WantedState.PASSING);
              shooter.setWantedState(Shooter.WantedState.PASSING);
              turret.setWantedState(Turret.WantedState.PASSING);

              spindexer.setWantedState(Spindexer.WantedState.IDLE);
              kicker.setWantedState(Kicker.WantedState.IDLE);

              state.setSuperstructureState(GenericSuperstructureState.WAITING);
            })
        .until(() -> shooter.getSetpointRPM() > 0 && shooter.atSetpoint());
  }

  private Command requestShootingToPass() {
    return this.run(
        () -> {
          hood.setWantedState(Hood.WantedState.PASSING);
          shooter.setWantedState(Shooter.WantedState.PASSING);
          turret.setWantedState(Turret.WantedState.PASSING);

          // hood will never change and shooter is allowed to fluctuate, mainly we are concerned
          // about turret

          if (turret.atSetpoint() && !state.passingWillHitHub()) {
            spindexer.setWantedState(Spindexer.WantedState.SPIN);
            kicker.setWantedState(Kicker.WantedState.KICK);

            state.setSuperstructureState(GenericSuperstructureState.PASS);
          } else {
            spindexer.setWantedState(Spindexer.WantedState.IDLE);
            kicker.setWantedState(Kicker.WantedState.IDLE);

            state.setSuperstructureState(GenericSuperstructureState.WAITING);
          }
        });
  }

  public Command autoSpinUpIndefinite() {
    return this.run(
        () -> {
          hood.setWantedState(Hood.WantedState.IDLE);
          shooter.setWantedState(Shooter.WantedState.SCORING);
          turret.setWantedState(Turret.WantedState.SCORING);

          spindexer.setWantedState(Spindexer.WantedState.IDLE);
          kicker.setWantedState(Kicker.WantedState.IDLE);

          state.setSuperstructureState(GenericSuperstructureState.WAITING);
        });
  }

  public Command manual4() {
    var setpoint =
        ShooterScoringData.shooterLookupTable.calculateShooterStateStationary(
            new Pose2d(Units.inchesToMeters(17), Units.inchesToMeters(17), Rotation2d.kZero)
                .plus(
                    new Transform2d(
                            Constants.SimConstants.originToTurretRotation.getX(),
                            0,
                            Rotation2d.kZero)
                        .inverse()),
            FieldConstants.Hub.topCenterPoint.toTranslation2d());
    return this.runOnce(() -> disableTurret = true)
        .andThen(
            this.run(
                () -> {
                  shooter.setMANUAL_RPM(setpoint.shooterRPM().in(Rotations.per(Minute)));
                  shooter.setWantedState(Shooter.WantedState.MANUAL);
                  hood.setMANUAL_HOOD_ANGLE(setpoint.hoodAngle().in(Degrees));
                  hood.setWantedState(Hood.WantedState.MANUAL);

                  turret.setWantedState(Turret.WantedState.IDLE);

                  spindexer.setWantedState(Spindexer.WantedState.IDLE);
                  kicker.setWantedState(Kicker.WantedState.IDLE);
                }))
        .until(() -> shooter.atSetpoint() && shooter.getSetpointRPM() > 0)
        .andThen(
            this.run(
                () -> {
                  shooter.setMANUAL_RPM(setpoint.shooterRPM().in(Rotations.per(Minute)));
                  shooter.setWantedState(Shooter.WantedState.MANUAL);
                  hood.setMANUAL_HOOD_ANGLE(setpoint.hoodAngle().in(Degrees));
                  hood.setWantedState(Hood.WantedState.MANUAL);

                  turret.setWantedState(Turret.WantedState.IDLE);

                  spindexer.setWantedState(Spindexer.WantedState.SPIN);
                  kicker.setWantedState(Kicker.WantedState.KICK);
                }));
  }

  public Command manual3() {
    var setpoint =
        ShooterScoringData.shooterLookupTable.calculateShooterStateStationary(
            new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero)
                .plus(
                    new Transform2d(1.97 + (4.9 - 1.97) * 3.0 / 4.0, 0, Rotation2d.kZero)
                        .inverse()),
            FieldConstants.Hub.topCenterPoint.toTranslation2d());
    return this.runOnce(() -> disableTurret = true)
        .andThen(
            this.run(
                () -> {
                  shooter.setMANUAL_RPM(setpoint.shooterRPM().in(Rotations.per(Minute)));
                  shooter.setWantedState(Shooter.WantedState.MANUAL);
                  hood.setMANUAL_HOOD_ANGLE(setpoint.hoodAngle().in(Degrees));
                  hood.setWantedState(Hood.WantedState.MANUAL);

                  turret.setWantedState(Turret.WantedState.IDLE);

                  spindexer.setWantedState(Spindexer.WantedState.IDLE);
                  kicker.setWantedState(Kicker.WantedState.IDLE);
                }))
        .until(() -> shooter.atSetpoint() && shooter.getSetpointRPM() > 0)
        .andThen(
            this.run(
                () -> {
                  shooter.setMANUAL_RPM(setpoint.shooterRPM().in(Rotations.per(Minute)));
                  shooter.setWantedState(Shooter.WantedState.MANUAL);
                  hood.setMANUAL_HOOD_ANGLE(setpoint.hoodAngle().in(Degrees));
                  hood.setWantedState(Hood.WantedState.MANUAL);

                  turret.setWantedState(Turret.WantedState.IDLE);

                  spindexer.setWantedState(Spindexer.WantedState.SPIN);
                  kicker.setWantedState(Kicker.WantedState.KICK);
                }));
  }

  public Command manual2() {
    var setpoint =
        ShooterScoringData.shooterLookupTable.calculateShooterStateStationary(
            new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero)
                .plus(
                    new Transform2d(1.97 + (4.9 - 1.97) * 2.0 / 4.0, 0, Rotation2d.kZero)
                        .inverse()),
            FieldConstants.Hub.topCenterPoint.toTranslation2d());
    return this.runOnce(() -> disableTurret = true)
        .andThen(
            this.run(
                () -> {
                  shooter.setMANUAL_RPM(setpoint.shooterRPM().in(Rotations.per(Minute)));
                  shooter.setWantedState(Shooter.WantedState.MANUAL);
                  hood.setMANUAL_HOOD_ANGLE(setpoint.hoodAngle().in(Degrees));
                  hood.setWantedState(Hood.WantedState.MANUAL);

                  turret.setWantedState(Turret.WantedState.IDLE);

                  spindexer.setWantedState(Spindexer.WantedState.IDLE);
                  kicker.setWantedState(Kicker.WantedState.IDLE);
                }))
        .until(() -> shooter.atSetpoint() && shooter.getSetpointRPM() > 0)
        .andThen(
            this.run(
                () -> {
                  shooter.setMANUAL_RPM(setpoint.shooterRPM().in(Rotations.per(Minute)));
                  shooter.setWantedState(Shooter.WantedState.MANUAL);
                  hood.setMANUAL_HOOD_ANGLE(setpoint.hoodAngle().in(Degrees));
                  hood.setWantedState(Hood.WantedState.MANUAL);

                  turret.setWantedState(Turret.WantedState.IDLE);

                  spindexer.setWantedState(Spindexer.WantedState.SPIN);
                  kicker.setWantedState(Kicker.WantedState.KICK);
                }));
  }

  public Command manual1() {
    var setpoint =
        ShooterScoringData.shooterLookupTable.calculateShooterStateStationary(
            new Pose2d(FieldConstants.Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero)
                .plus(
                    new Transform2d(1.97 + (4.9 - 1.97) * 1.0 / 4.0, 0, Rotation2d.kZero)
                        .inverse()),
            FieldConstants.Hub.topCenterPoint.toTranslation2d());
    return this.runOnce(() -> disableTurret = true)
        .andThen(
            this.run(
                () -> {
                  shooter.setMANUAL_RPM(setpoint.shooterRPM().in(Rotations.per(Minute)));
                  shooter.setWantedState(Shooter.WantedState.MANUAL);
                  hood.setMANUAL_HOOD_ANGLE(setpoint.hoodAngle().in(Degrees));
                  hood.setWantedState(Hood.WantedState.MANUAL);

                  turret.setWantedState(Turret.WantedState.IDLE);

                  spindexer.setWantedState(Spindexer.WantedState.IDLE);
                  kicker.setWantedState(Kicker.WantedState.IDLE);
                }))
        .until(() -> shooter.atSetpoint() && shooter.getSetpointRPM() > 0)
        .andThen(
            this.run(
                () -> {
                  shooter.setMANUAL_RPM(setpoint.shooterRPM().in(Rotations.per(Minute)));
                  shooter.setWantedState(Shooter.WantedState.MANUAL);
                  hood.setMANUAL_HOOD_ANGLE(setpoint.hoodAngle().in(Degrees));
                  hood.setWantedState(Hood.WantedState.MANUAL);

                  turret.setWantedState(Turret.WantedState.IDLE);

                  spindexer.setWantedState(Spindexer.WantedState.SPIN);
                  kicker.setWantedState(Kicker.WantedState.KICK);
                }));
  }

  public InstantCommand toggleTurretDisable() {
    return new InstantCommand(() -> disableTurret = !disableTurret);
  }

  public enum GenericSuperstructureState {
    IDLE,
    WAITING,
    SHOOT,
    PASS,
    MANUAL
  }
}
