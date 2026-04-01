/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.team4639.frc2026.FieldConstants;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.commands.CosineAlignToY;
import org.team4639.frc2026.commands.DriveCommands;
import org.team4639.frc2026.commands.IntakeCommands;
import org.team4639.frc2026.commands.SuperstructureCommands;
import org.team4639.frc2026.subsystems.drive.Drive;
import org.team4639.frc2026.subsystems.extension.Extension;
import org.team4639.frc2026.subsystems.hood.Hood;
import org.team4639.frc2026.subsystems.intake.Intake;
import org.team4639.frc2026.subsystems.kicker.Kicker;
import org.team4639.frc2026.subsystems.shooter.Shooter;
import org.team4639.frc2026.subsystems.spindexer.Spindexer;
import org.team4639.frc2026.subsystems.turret.Turret;

public class AutoCommands4 {

  public static final double INTAKE_AFTER_MOVING_METERS = 0.5;
  public static final Pose2d LEFT_SHOOT_SETPOINT = new Pose2d(2.818, 7.240, Rotation2d.k180deg);
  public static final Pose2d LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE =
      new Pose2d(
          FieldConstants.LinesVertical.hubCenter,
          (FieldConstants.LinesHorizontal.leftTrenchOpenEnd
                  + FieldConstants.LinesHorizontal.leftTrenchOpenStart)
              / 2.0,
          Rotation2d.k180deg);

  public static final Pose2d LEFT_TRENCH_ALIGN_SETPOINT_TO_NEUTRAL_ZONE =
      new Pose2d(
          LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE.getX() + 1,
          LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE.getY(),
          Rotation2d.kCW_90deg);

  public static final Pose2d LEFT_BUMP_ALIGN_SETPOINT_45 =
      new Pose2d(
          FieldConstants.LinesVertical.allianceZone - 2,
          (FieldConstants.LinesHorizontal.leftBumpStart
                  + FieldConstants.LinesHorizontal.leftBumpEnd)
              / 2.0,
          Rotation2d.fromDegrees(45));
  public static final Pose2d LEFT_BUMP_ALIGN_SETPOINT_NEGATIVE_45 =
      LEFT_BUMP_ALIGN_SETPOINT_45.rotateBy(Rotation2d.kCW_90deg);
  public static final int TRENCH_COS_POWER = 100;
  public static final double BUMP_SPEED = 26.0;
  public static final int BUMP_COS_POWER = 1000;
  public static final double TRENCH_SPEED = 9.0;

  public static Command OP_LEFT(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPath("OPL-0", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.stop(intake), IntakeCommands.retract(extension))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.neutralZoneNear + 0.5),
                new ParallelCommandGroup(
                    IntakeCommands.autoForceExtension(extension), IntakeCommands.intake(intake))),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state)),
        new ParallelDeadlineGroup(
            followPath("OPL-1", false),
            Commands.runOnce(
                () -> {
                  state.resetPose(state.getSecondaryEstimatedPose());
                  state.setSendVisionToPrimaryPoseEstimator(true);
                }),
            new SequentialCommandGroup(
                SuperstructureCommands.autoSpinupToShoot(
                        shooter, hood, turret, spindexer, kicker, state)
                    .alongWith(IntakeCommands.autoDrawInExtension(extension))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                    < FieldConstants.LinesVertical.allianceZone
                                && state.getTurretToGoal() > 1.87),
                SuperstructureCommands.requestScoring(
                        shooter, hood, turret, spindexer, kicker, state)
                    .alongWith(IntakeCommands.autoForceExtension(extension)))),
        IntakeCommands.intake(intake),
        new ParallelDeadlineGroup(
            followPath("OPL-2", false),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.stop(intake), IntakeCommands.retract(extension))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.neutralZoneNear + 0.5),
                new ParallelCommandGroup(
                    IntakeCommands.autoForceExtension(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            followPath("OPL-3", false),
            Commands.runOnce(
                () -> {
                  state.resetPose(state.getSecondaryEstimatedPose());
                  state.setSendVisionToPrimaryPoseEstimator(true);
                }),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.intake(intake),
            IntakeCommands.autoDrawInExtension(extension)),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            IntakeCommands.intake(intake),
            IntakeCommands.autoForceExtension(extension),
            SuperstructureCommands.requestScoring(
                shooter, hood, turret, spindexer, kicker, state)));
  }

  public static Command OP_RIGHT(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPathMirrored("OPL-0", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.stop(intake), IntakeCommands.retract(extension))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.neutralZoneNear + 0.5),
                new ParallelCommandGroup(
                    IntakeCommands.autoForceExtension(extension), IntakeCommands.intake(intake))),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state)),
        new ParallelDeadlineGroup(
            followPathMirrored("OPL-1", false),
            Commands.runOnce(
                () -> {
                  state.resetPose(state.getSecondaryEstimatedPose());
                  state.setSendVisionToPrimaryPoseEstimator(true);
                }),
            new SequentialCommandGroup(
                SuperstructureCommands.autoSpinupToShoot(
                        shooter, hood, turret, spindexer, kicker, state)
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                    < FieldConstants.LinesVertical.allianceZone - 0.25
                                && state.getTurretToGoal() > 2.25),
                SuperstructureCommands.requestScoring(
                    shooter, hood, turret, spindexer, kicker, state)),
            IntakeCommands.intakeAgitate(intake),
            IntakeCommands.autoDrawInExtension(extension)),
        new ParallelDeadlineGroup(
            followPathMirrored("OPL-2", false),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.stop(intake), IntakeCommands.retract(extension))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.neutralZoneNear + 0.5),
                new ParallelCommandGroup(
                    IntakeCommands.autoForceExtension(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            followPathMirrored("OPL-3", false),
            Commands.runOnce(
                () -> {
                  state.resetPose(state.getSecondaryEstimatedPose());
                  state.setSendVisionToPrimaryPoseEstimator(true);
                }),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.stop(intake),
            IntakeCommands.autoDrawInExtension(extension)),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            IntakeCommands.intakeAgitate(intake),
            IntakeCommands.autoDrawInExtension(extension),
            SuperstructureCommands.requestScoring(
                shooter, hood, turret, spindexer, kicker, state)));
  }

  public static Command LEFT_SINGLE_SWIPE(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPath("LS_CYCF", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            followPath("CYCF_LSH", false, state),
            Commands.runOnce(
                () -> {
                  state.resetPose(state.getSecondaryEstimatedPose());
                  state.setSendVisionToPrimaryPoseEstimator(true);
                }),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.extend(extension),
            IntakeCommands.intake(intake)),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            SuperstructureCommands.requestScoring(
                shooter, hood, turret, spindexer, kicker, state)));
  }

  public static Command RIGHT_SINGLE_SWIPE(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPathMirrored("LS_CYCF", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            followPathMirrored("CYCF_LSH", false, state),
            Commands.runOnce(
                () -> {
                  state.resetPose(state.getSecondaryEstimatedPose());
                  state.setSendVisionToPrimaryPoseEstimator(true);
                }),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.agitate(extension, intake)));
  }

  public static Command LEFT_FAR_SWIPE(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPath("LS_CYCFF", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                drive, LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE, TRENCH_SPEED, TRENCH_COS_POWER),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelDeadlineGroup(
            drive.defer(() -> DriveCommands.PIDToPose(drive, state, LEFT_SHOOT_SETPOINT, 0.25)),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.agitate(extension, intake)));
  }

  public static Command RIGHT_FAR_SWIPE(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPathMirrored("LS_CYCFF", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                drive,
                mirror(LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE),
                TRENCH_SPEED,
                TRENCH_COS_POWER),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelDeadlineGroup(
            drive.defer(
                () -> DriveCommands.PIDToPose(drive, state, mirror(LEFT_SHOOT_SETPOINT), 0.25)),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.agitate(extension, intake)));
  }

  public static Command LEFT_DOUBLE_SWIPE(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPath("LS_CYCF", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                drive, LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE, TRENCH_SPEED, TRENCH_COS_POWER),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelDeadlineGroup(
            drive.defer(() -> DriveCommands.PIDToPose(drive, state, LEFT_SHOOT_SETPOINT, 0.25)),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
                DriveCommands.joystickDriveAtAngle(
                    drive, () -> 0, () -> 0, () -> Rotation2d.k180deg),
                SuperstructureCommands.requestScoring(
                    shooter, hood, turret, spindexer, kicker, state),
                IntakeCommands.agitate(extension, intake))
            .withTimeout(4),
        new ParallelDeadlineGroup(
            followPath("LSH_CYCM", false, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                drive, LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE, TRENCH_SPEED, TRENCH_COS_POWER),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelDeadlineGroup(
            drive.defer(() -> DriveCommands.PIDToPose(drive, state, LEFT_SHOOT_SETPOINT, 0.25)),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.agitate(extension, intake)));
  }

  public static Command RIGHT_DOUBLE_SWIPE(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPathMirrored("LS_CYCF", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                drive,
                mirror(LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE),
                TRENCH_SPEED,
                TRENCH_COS_POWER),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelDeadlineGroup(
            drive.defer(
                () -> DriveCommands.PIDToPose(drive, state, mirror(LEFT_SHOOT_SETPOINT), 0.25)),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
                drive.run(drive::stopWithX),
                SuperstructureCommands.requestScoring(
                    shooter, hood, turret, spindexer, kicker, state),
                IntakeCommands.agitate(extension, intake))
            .withTimeout(4),
        new ParallelDeadlineGroup(
            followPathMirrored("LSH_CYCM", false, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                drive,
                mirror(LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE),
                TRENCH_SPEED,
                TRENCH_COS_POWER),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelDeadlineGroup(
            drive.defer(
                () -> DriveCommands.PIDToPose(drive, state, mirror(LEFT_SHOOT_SETPOINT), 0.25)),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.agitate(extension, intake)));
  }

  public static Command LEFT_FAR_DOUBLE_SWIPE(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPath("LS_CYCFF", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                drive, LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE, TRENCH_SPEED, TRENCH_COS_POWER),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelDeadlineGroup(
            drive.defer(() -> DriveCommands.PIDToPose(drive, state, LEFT_SHOOT_SETPOINT, 0.25)),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
                drive.run(drive::stopWithX),
                SuperstructureCommands.requestScoring(
                    shooter, hood, turret, spindexer, kicker, state),
                IntakeCommands.agitate(extension, intake))
            .withTimeout(4),
        new ParallelDeadlineGroup(
            followPath("LSH_CYCM", false, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                drive, LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE, TRENCH_SPEED, TRENCH_COS_POWER),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelDeadlineGroup(
            drive.defer(() -> DriveCommands.PIDToPose(drive, state, LEFT_SHOOT_SETPOINT, 0.25)),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.agitate(extension, intake)));
  }

  public static Command RIGHT_FAR_DOUBLE_SWIPE(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPathMirrored("LS_CYCFF", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                drive,
                mirror(LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE),
                TRENCH_SPEED,
                TRENCH_COS_POWER),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelDeadlineGroup(
            drive.defer(
                () -> DriveCommands.PIDToPose(drive, state, mirror(LEFT_SHOOT_SETPOINT), 0.25)),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
                drive.run(drive::stopWithX),
                SuperstructureCommands.requestScoring(
                    shooter, hood, turret, spindexer, kicker, state),
                IntakeCommands.agitate(extension, intake))
            .withTimeout(4),
        new ParallelDeadlineGroup(
            followPathMirrored("LSH_CYCM", false, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                drive,
                mirror(LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE),
                TRENCH_SPEED,
                TRENCH_COS_POWER),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelDeadlineGroup(
            drive.defer(
                () -> DriveCommands.PIDToPose(drive, state, mirror(LEFT_SHOOT_SETPOINT), 0.25)),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.agitate(extension, intake)));
  }

  public static Command LEFT_CHEESY_SINGLE_SWIPE(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPath("LS_CHZ", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                drive, LEFT_TRENCH_ALIGN_SETPOINT_TO_ALLIANCE_ZONE, TRENCH_SPEED, TRENCH_COS_POWER),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelDeadlineGroup(
            drive.defer(() -> DriveCommands.PIDToPose(drive, state, LEFT_SHOOT_SETPOINT, 0.25)),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.agitate(extension, intake)));
  }

  public static Command LEFT_CITRUS_SINGLE_SWIPE(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPath("LS_CHZ", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(drive, LEFT_BUMP_ALIGN_SETPOINT_45, BUMP_SPEED, BUMP_COS_POWER)
                .until(
                    () ->
                        state.getEstimatedPose().getX()
                            < FieldConstants.LinesVertical.starting - 0.5),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.agitate(extension, intake)));
  }

  public static Command LEFT_CITRUS_DOUBLE_SWIPE(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPath("LS_CHZ", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(drive, LEFT_BUMP_ALIGN_SETPOINT_45, BUMP_SPEED, BUMP_COS_POWER)
                .until(
                    () ->
                        state.getEstimatedPose().getX()
                            < FieldConstants.LinesVertical.starting - 1),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
                DriveCommands.joystickDriveAtAngle(
                    drive, () -> 0, () -> 0, () -> Rotation2d.kCCW_90deg),
                SuperstructureCommands.requestScoring(
                    shooter, hood, turret, spindexer, kicker, state),
                IntakeCommands.agitate(extension, intake))
            .withTimeout(4),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                    drive,
                    LEFT_TRENCH_ALIGN_SETPOINT_TO_NEUTRAL_ZONE,
                    TRENCH_SPEED,
                    TRENCH_COS_POWER)
                .until(
                    () ->
                        state.getEstimatedPose().getX()
                            > FieldConstants.LinesVertical.hubCenter + 0.5),
            IntakeCommands.stop(intake),
            IntakeCommands.retract(extension),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state)),
        new ParallelCommandGroup(
            followPath("LTR_CHZ2", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(drive, LEFT_BUMP_ALIGN_SETPOINT_45, BUMP_SPEED, BUMP_COS_POWER)
                .until(
                    () ->
                        state.getEstimatedPose().getX()
                            < FieldConstants.LinesVertical.starting - 0.5),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> 0, () -> 0, () -> Rotation2d.kCCW_90deg),
            SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.agitate(extension, intake)));
  }

  public static Command RIGHT_CITRUS_DOUBLE_SWIPE(
      Drive drive,
      Shooter shooter,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Kicker kicker,
      Extension extension,
      Intake intake,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPathMirrored("LS_CHZ", true, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                    drive, mirror(LEFT_BUMP_ALIGN_SETPOINT_45), BUMP_SPEED, BUMP_COS_POWER)
                .until(
                    () ->
                        state.getEstimatedPose().getX()
                            < FieldConstants.LinesVertical.starting - 1),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
                drive.run(drive::stopWithX),
                SuperstructureCommands.requestScoring(
                    shooter, hood, turret, spindexer, kicker, state),
                IntakeCommands.agitate(extension, intake))
            .withTimeout(4),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                    drive,
                    mirror(LEFT_TRENCH_ALIGN_SETPOINT_TO_NEUTRAL_ZONE),
                    TRENCH_SPEED,
                    TRENCH_COS_POWER)
                .until(
                    () -> state.getEstimatedPose().getX() > FieldConstants.LinesVertical.hubCenter),
            IntakeCommands.stop(intake),
            IntakeCommands.retract(extension),
            SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state)),
        new ParallelCommandGroup(
            followPathMirrored("LTR_CHZ2", false, state),
            Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                        IntakeCommands.retract(extension), IntakeCommands.stop(intake))
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.hubCenter
                                    + INTAKE_AFTER_MOVING_METERS),
                new ParallelCommandGroup(
                    IntakeCommands.extend(extension), IntakeCommands.intake(intake)))),
        new ParallelDeadlineGroup(
            new CosineAlignToY(
                    drive, mirror(LEFT_BUMP_ALIGN_SETPOINT_45), BUMP_SPEED, BUMP_COS_POWER)
                .until(
                    () ->
                        state.getEstimatedPose().getX()
                            < FieldConstants.LinesVertical.starting - 0.5),
            SuperstructureCommands.autoSpinupToShoot(
                shooter, hood, turret, spindexer, kicker, state),
            Commands.runOnce(
                () -> {
                  state.setSendVisionToPrimaryPoseEstimator(true);
                  state.resetPose(state.getSecondaryEstimatedPose());
                }),
            new ParallelCommandGroup(
                IntakeCommands.extend(extension), IntakeCommands.intake(intake))),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state),
            IntakeCommands.agitate(extension, intake)));
  }

  private static Command followPath(String pathName, boolean resetPose, RobotState... state) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      Command beforeStarting = Commands.none();
      if (resetPose) {
        if (state.length > 0) {
          beforeStarting =
              Commands.runOnce(() -> state[0].resetPose(path.getStartingHolonomicPose().get()));
        }
      }
      return beforeStarting.andThen(AutoBuilder.followPath(path));
    } catch (IOException e) {
      throw new RuntimeException(e);
    } catch (ParseException e) {
      throw new RuntimeException(e);
    }
  }

  private static Command followPathMirrored(
      String pathName, boolean resetPose, RobotState... state) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName).mirrorPath();
      Command beforeStarting = Commands.none();
      if (resetPose) {
        if (state.length > 0) {
          beforeStarting =
              Commands.runOnce(() -> state[0].resetPose(path.getStartingHolonomicPose().get()));
        }
      }
      return beforeStarting.andThen(AutoBuilder.followPath(path));
    } catch (IOException e) {
      throw new RuntimeException(e);
    } catch (ParseException e) {
      throw new RuntimeException(e);
    }
  }

  private static Pose2d mirror(Pose2d pose) {
    return new Pose2d(
        pose.getX(), FieldConstants.fieldWidth - pose.getY(), pose.getRotation().unaryMinus());
  }
}
