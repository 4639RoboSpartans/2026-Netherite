/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.auto;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.team4639.frc2026.FieldConstants;
import org.team4639.frc2026.RobotState;
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

public class AutoCommands {

    public static final double EXTEND_INTAKE_METERS_PAST_LINE = 0.5;

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
                                new ParallelCommandGroup(IntakeCommands.stop(intake), IntakeCommands.retract(extension))
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                        + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        IntakeCommands.autoForceExtension(extension), IntakeCommands.intake(intake))),
                        SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state)),
                new ParallelDeadlineGroup(
                        followPath("OPL-1", false, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        new SequentialCommandGroup(
                                SuperstructureCommands.autoSpinupToShoot(
                                                shooter, hood, turret, spindexer, kicker, state)
                                        .alongWith(IntakeCommands.autoDrawInExtension(extension))
                                        .until(() -> state.getEstimatedPose().getX()
                                                        < FieldConstants.LinesVertical.allianceZone
                                                && state.getTurretToGoal() > 1.87),
                                SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state)
                                        .alongWith(IntakeCommands.agitate(extension, intake)))),
                IntakeCommands.intake(intake),
                new ParallelDeadlineGroup(
                        followPath("OPL-2", false, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(IntakeCommands.stop(intake), IntakeCommands.retract(extension))
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                        + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        IntakeCommands.autoForceExtension(extension), IntakeCommands.intake(intake)))),
                new ParallelDeadlineGroup(
                        followPath("OPL-3", false, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        SuperstructureCommands.autoSpinupToShoot(shooter, hood, turret, spindexer, kicker, state),
                        IntakeCommands.intake(intake),
                        IntakeCommands.autoDrawInExtension(extension)),
                new ParallelCommandGroup(
                        drive.run(drive::stopWithX),
                        IntakeCommands.agitate(extension, intake),
                        SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state)));
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
                                new ParallelCommandGroup(IntakeCommands.stop(intake), IntakeCommands.retract(extension))
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                        + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        IntakeCommands.autoForceExtension(extension), IntakeCommands.intake(intake))),
                        SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state)),
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-1", false, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        new SequentialCommandGroup(
                                SuperstructureCommands.autoSpinupToShoot(
                                                shooter, hood, turret, spindexer, kicker, state)
                                        .alongWith(IntakeCommands.autoDrawInExtension(extension))
                                        .until(() -> state.getEstimatedPose().getX()
                                                        < FieldConstants.LinesVertical.allianceZone
                                                && state.getTurretToGoal() > 1.87),
                                SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state)
                                        .alongWith(IntakeCommands.agitate(extension, intake)))),
                IntakeCommands.intake(intake),
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-2", false, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(IntakeCommands.stop(intake), IntakeCommands.retract(extension))
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                        + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        IntakeCommands.autoForceExtension(extension), IntakeCommands.intake(intake)))),
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-3", false, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        SuperstructureCommands.autoSpinupToShoot(shooter, hood, turret, spindexer, kicker, state),
                        IntakeCommands.intake(intake),
                        IntakeCommands.autoDrawInExtension(extension)),
                new ParallelCommandGroup(
                        drive.run(drive::stopWithX),
                        IntakeCommands.agitate(extension, intake),
                        SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state)));
    }

    public static Command OP_NEAR_LEFT(
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
                        followPath("OPNL-0", true, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(IntakeCommands.stop(intake), IntakeCommands.retract(extension))
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                        + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        IntakeCommands.autoForceExtension(extension), IntakeCommands.intake(intake))),
                        SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state)),
                new ParallelDeadlineGroup(
                        followPath("OPL-1", false, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        new SequentialCommandGroup(
                                SuperstructureCommands.autoSpinupToShoot(
                                                shooter, hood, turret, spindexer, kicker, state)
                                        .alongWith(IntakeCommands.autoDrawInExtension(extension))
                                        .until(() -> state.getEstimatedPose().getX()
                                                        < FieldConstants.LinesVertical.allianceZone
                                                && state.getTurretToGoal() > 1.87),
                                SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state)
                                        .alongWith(IntakeCommands.agitate(extension, intake)))),
                IntakeCommands.intake(intake),
                new ParallelDeadlineGroup(
                        followPath("OPL-2", false, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(IntakeCommands.stop(intake), IntakeCommands.retract(extension))
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                        + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        IntakeCommands.autoForceExtension(extension), IntakeCommands.intake(intake)))),
                new ParallelDeadlineGroup(
                        followPath("OPL-3", false, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        SuperstructureCommands.autoSpinupToShoot(shooter, hood, turret, spindexer, kicker, state),
                        IntakeCommands.intake(intake),
                        IntakeCommands.autoDrawInExtension(extension)),
                new ParallelCommandGroup(
                        drive.run(drive::stopWithX),
                        IntakeCommands.agitate(extension, intake),
                        SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state)));
    }

    public static Command OP_NEAR_RIGHT(
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
                        followPathMirrored("OPNL-0", true, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(IntakeCommands.stop(intake), IntakeCommands.retract(extension))
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                        + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        IntakeCommands.autoForceExtension(extension), IntakeCommands.intake(intake))),
                        SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state)),
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-1", false, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        new SequentialCommandGroup(
                                SuperstructureCommands.autoSpinupToShoot(
                                                shooter, hood, turret, spindexer, kicker, state)
                                        .alongWith(IntakeCommands.autoDrawInExtension(extension))
                                        .until(() -> state.getEstimatedPose().getX()
                                                        < FieldConstants.LinesVertical.allianceZone
                                                && state.getTurretToGoal() > 1.87),
                                SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state)
                                        .alongWith(IntakeCommands.agitate(extension, intake)))),
                IntakeCommands.intake(intake),
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-2", false, state),
                        Commands.runOnce(() -> state.setSendVisionToPrimaryPoseEstimator(false)),
                        SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, state),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(IntakeCommands.stop(intake), IntakeCommands.retract(extension))
                                        .until(() -> state.getEstimatedPose().getX()
                                                > FieldConstants.LinesVertical.neutralZoneNear
                                                        + EXTEND_INTAKE_METERS_PAST_LINE),
                                new ParallelCommandGroup(
                                        IntakeCommands.autoForceExtension(extension), IntakeCommands.intake(intake)))),
                new ParallelDeadlineGroup(
                        followPathMirrored("OPL-3", false, state),
                        Commands.runOnce(() -> {
                            state.resetPose(state.getSecondaryEstimatedPose());
                            state.setSendVisionToPrimaryPoseEstimator(true);
                        }),
                        SuperstructureCommands.autoSpinupToShoot(shooter, hood, turret, spindexer, kicker, state),
                        IntakeCommands.intake(intake),
                        IntakeCommands.autoDrawInExtension(extension)),
                new ParallelCommandGroup(
                        drive.run(drive::stopWithX),
                        IntakeCommands.agitate(extension, intake),
                        SuperstructureCommands.requestScoring(shooter, hood, turret, spindexer, kicker, state)));
    }

    /**
     * Follow PathPlanner path. If this is the start of an auto routine, then reset
     * pose to the starting pose of the path. Otherwise, pathfind to the start of
     * the path.
     *
     * @param pathName
     *            name of the Path in PathPlanner
     * @param resetPose
     *            whether to reset the pose to the path's starting pose.
     */
    private static Command followPath(String pathName, boolean resetPose, RobotState state) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            Command beforeStarting = Commands.none();
            if (resetPose) {
                beforeStarting = Commands.runOnce(
                        () -> state.resetPose(path.getStartingHolonomicPose().get()));
            }
            var lastTranslation =
                    path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation();

            if (resetPose)
                return beforeStarting
                        .andThen(AutoBuilder.followPath(path))
                        .until(() -> state.getEstimatedPose().getTranslation().getDistance(lastTranslation) < 0.1);

            return beforeStarting
                    .andThen(AutoBuilder.pathfindThenFollowPath(
                            path,
                            new PathConstraints(
                                    MetersPerSecond.of(5),
                                    MetersPerSecondPerSecond.of(2),
                                    DegreesPerSecond.of(540),
                                    DegreesPerSecondPerSecond.of(720))))
                    .until(() -> state.getEstimatedPose().getTranslation().getDistance(lastTranslation) < 0.1);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * {@link AutoCommands#followPath(String, boolean, RobotState)} but mirrors the
     * path from left to right or vice versa.
     */
    private static Command followPathMirrored(String pathName, boolean resetPose, RobotState state) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName).mirrorPath();
            Command beforeStarting = Commands.none();
            if (resetPose) {
                beforeStarting = Commands.runOnce(
                        () -> state.resetPose(path.getStartingHolonomicPose().get()));
            }
            var lastTranslation =
                    path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation();

            if (resetPose)
                return beforeStarting
                        .andThen(AutoBuilder.followPath(path))
                        .until(() -> state.getEstimatedPose().getTranslation().getDistance(lastTranslation) < 0.1);
            return beforeStarting
                    .andThen(AutoBuilder.pathfindThenFollowPath(
                            path,
                            new PathConstraints(
                                    MetersPerSecond.of(5),
                                    MetersPerSecondPerSecond.of(2),
                                    DegreesPerSecond.of(540),
                                    DegreesPerSecondPerSecond.of(720))))
                    .until(() -> state.getEstimatedPose().getTranslation().getDistance(lastTranslation) < 0.1);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
