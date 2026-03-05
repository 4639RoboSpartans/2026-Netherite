package org.team4639.frc2026.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import org.json.simple.parser.ParseException;
import org.team4639.frc2026.FieldConstants;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.subsystems.IntakeStructure;
import org.team4639.frc2026.subsystems.Superstructure;
import org.team4639.frc2026.subsystems.drive.Drive;

import java.io.IOException;

public class AutoCommands3 {
    private Command LEFT_DOUBLE_SWIPE(Drive drive, Superstructure superstructure, IntakeStructure intakeStructure, RobotState state) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        followPath(AutoPaths.LS_CYCF, true, state),
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
                        superstructure.trackHub()
                ),
                new ParallelCommandGroup(
                        followPath(AutoPaths.CYCF_LSH, false, state),
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
                ).withTimeout(4),
                new ParallelCommandGroup(
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
                        superstructure.trackHub()
                ),
                new ParallelCommandGroup(
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
                ).withTimeout(4)
        );
    }

    private Command followPath(String pathName, boolean resetPose, RobotState state){
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return AutoBuilder.followPath(path)
                    .beforeStarting(
                            resetPose ?
                                    new InstantCommand(() -> state.resetPose(path.getStartingHolonomicPose().get()))
                                    : Commands.none()
                    );
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    private Command followPathMirrored(String pathName, boolean resetPose, RobotState state){
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName).mirrorPath();
            return AutoBuilder.followPath(path)
                    .beforeStarting(
                            resetPose ?
                                    new InstantCommand(() -> state.resetPose(path.getStartingHolonomicPose().get()))
                                    : Commands.none()
                    );
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
    }
}
