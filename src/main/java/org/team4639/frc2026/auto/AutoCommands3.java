/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.team4639.frc2026.FieldConstants;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.subsystems.IntakeStructure;
import org.team4639.frc2026.subsystems.Superstructure;
import org.team4639.frc2026.subsystems.drive.Drive;
import org.team4639.frc2026.subsystems.vision.Vision;

public class AutoCommands3 {
  public static Command LEFT_DOUBLE_SWIPE(
      Drive drive,
      Superstructure superstructure,
      IntakeStructure intakeStructure,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPath(AutoPaths.LS_CYCF, true, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(intakeStructure.stopIntake(), intakeStructure.retract())
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.neutralZoneNear + 0.75),
                new ParallelCommandGroup(intakeStructure.intake(), intakeStructure.extend())),
            superstructure.trackHub(),
            Vision.visionOff()),
        new ParallelDeadlineGroup(
            followPath(AutoPaths.CYCF_LSH, false, state),
            intakeStructure.intake(),
            intakeStructure.extend(),
            superstructure.trackHub()),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            intakeStructure.agitate(),
            superstructure.requestScoring()) // .withTimeout(4),
        /*new ParallelDeadlineGroup(
                followPath(AutoPaths.LSH_CYCM, false, state),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                intakeStructure.stopIntake(),
                                intakeStructure.retract()
                        ).until(
                                () -> state.getEstimatedPose().getX() > FieldConstants.LinesVertical.neutralZoneNear + 1
                        ),
                        new ParallelCommandGroup(
                                intakeStructure.intake(),
                                intakeStructure.extend()
                        )
                ),
                superstructure.trackHub(),
                Vision.visionOff()
        ),
        new ParallelDeadlineGroup(
                followPath(AutoPaths.CYCM_LSH, false, state),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                intakeStructure.intake(),
                                intakeStructure.extend()
                        ).until(
                                () -> state.getEstimatedPose().getX() < FieldConstants.LinesVertical.neutralZoneNear + 1
                        ),
                        new ParallelCommandGroup(
                                intakeStructure.retract(),
                                intakeStructure.stopIntake()
                        )
                ),
                superstructure.trackHub()
        ),
        new ParallelCommandGroup(
                drive.run(drive::stopWithX),
                intakeStructure.agitate(),
                superstructure.requestScoring()
        ).withTimeout(4)*/
        );
  }

  public static Command RIGHT_DOUBLE_SWIPE(
      Drive drive,
      Superstructure superstructure,
      IntakeStructure intakeStructure,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPathMirrored(AutoPaths.LS_CYCF, true, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(intakeStructure.stopIntake(), intakeStructure.retract())
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.neutralZoneNear + 0.75),
                new ParallelCommandGroup(intakeStructure.intake(), intakeStructure.extend())),
            superstructure.trackHub(),
            Vision.visionOff()),
        new ParallelDeadlineGroup(
            followPathMirrored(AutoPaths.CYCF_LSH, false, state),
            intakeStructure.intake(),
            intakeStructure.extend(),
            superstructure.trackHub()),
        new ParallelCommandGroup(
            drive.run(drive::stopWithX),
            intakeStructure.agitate(),
            superstructure.requestScoring()));
  }

  public static Command RIGHT_SWIPE_OUTPOST(
      Drive drive,
      Superstructure superstructure,
      IntakeStructure intakeStructure,
      RobotState state) {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            followPathMirrored(AutoPaths.LS_CYCF, true, state),
            new SequentialCommandGroup(
                new ParallelCommandGroup(intakeStructure.stopIntake(), intakeStructure.retract())
                    .until(
                        () ->
                            state.getEstimatedPose().getX()
                                > FieldConstants.LinesVertical.neutralZoneNear + 0.75),
                new ParallelCommandGroup(intakeStructure.intake(), intakeStructure.extend())),
            superstructure.trackHub(),
            Vision.visionOff()),
        new ParallelDeadlineGroup(
                followPathMirrored(AutoPaths.CYCF_LMDP, false, state),
                intakeStructure.extend(),
                intakeStructure.intake(),
                superstructure.trackHub())
            .until(() -> state.getEstimatedPose().getX() < 3.568 + 0.1),
        new ParallelCommandGroup(
            drive
                .run(
                    () ->
                        drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                -0.6, 0, 0, state.getEstimatedPose().getRotation())))
                .until(() -> state.getEstimatedPose().getX() < 0.770)
                .andThen(drive.run(drive::stopWithX)),
            intakeStructure.extend(),
            intakeStructure.intake(),
            superstructure.requestScoring()));
  }

  private static Command followPath(String pathName, boolean resetPose, RobotState state) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      var waypoints = path.getWaypoints();
      Translation2d last = waypoints.get(waypoints.size() - 1).anchor();
      double toleranceMeters = 0.1;
      return AutoBuilder.followPath(path)
          .beforeStarting(
              resetPose
                  ? new InstantCommand(() -> state.resetPose(path.getStartingHolonomicPose().get()))
                  : Commands.none()); // .until(() ->
      // Math.abs(state.getEstimatedPose().getTranslation().getDistance(last)) < toleranceMeters);
    } catch (IOException e) {
      throw new RuntimeException(e);
    } catch (ParseException e) {
      throw new RuntimeException(e);
    }
  }

  private static Command followPathMirrored(String pathName, boolean resetPose, RobotState state) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName).mirrorPath();
      var waypoints = path.getWaypoints();
      Translation2d last = waypoints.get(waypoints.size() - 1).anchor();
      double toleranceMeters = 0.1;
      return AutoBuilder.followPath(path)
          .beforeStarting(
              resetPose
                  ? new InstantCommand(() -> state.resetPose(path.getStartingHolonomicPose().get()))
                  : Commands.none()); // .until(() ->
      // Math.abs(state.getEstimatedPose().getTranslation().getDistance(last)) < toleranceMeters);
    } catch (IOException e) {
      throw new RuntimeException(e);
    } catch (ParseException e) {
      throw new RuntimeException(e);
    }
  }

  public static final class AutoPaths {
    public static final String LS_CYCF = "LS_CYCF";
    public static final String CYCF_LSH = "CYCF_LSH";
    public static final String LSH_CYCM = "LSH_CYCM";
    public static final String CYCM_LSH = "CYCM_LSH";
    public static final String CYCF_LMDP = "CYCF_LMDP";
    public static final String LMDP_LOUTPOST = "LMDP_LOUTPOST";
  }
}
