/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.commands.DriveCommands;
import org.team4639.frc2026.subsystems.IntakeStructure;
import org.team4639.frc2026.subsystems.Superstructure;
import org.team4639.frc2026.subsystems.drive.Drive;

public class AutoCommands2 {

    public static Command LEFT_DOUBLE_SWIPE(Drive drive, Superstructure superstructure, IntakeStructure intakeStructure, RobotState state) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> state.resetPose(AutoPoses.LEFT_START)),
                LEFT_SWIPE_FAR(drive, superstructure, intakeStructure, state),
                LEFT_SWIPE_MID(drive, superstructure, intakeStructure, state)
        );
    }

    public static Command RIGHT_DOUBLE_SWIPE(Drive drive, Superstructure superstructure, IntakeStructure intakeStructure, RobotState state) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> state.resetPose(AutoPoses.RIGHT_START)),
                RIGHT_SWIPE_FAR(drive, superstructure, intakeStructure, state),
                RIGHT_SWIPE_MID(drive, superstructure, intakeStructure, state)
        );
    }


    private static Command LEFT_SWIPE_FAR(Drive drive, Superstructure superstructure, IntakeStructure intakeStructure, RobotState state) {
        return new SequentialCommandGroup(
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.LEFT_TRENCHLINE_NEUTRAL_FAR, 2.0))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.LEFT_SWIPE_ENDPOINT_FAR, 0.75, 0.25))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.LEFT_TRENCHLINE_NEUTRAL_MID_FAR, 1.75))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.LEFT_TRENCHLINE_TRENCH, 1.0))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.LEFT_TRENCHLINE_AZ_MID, 1.0))
                        .deadlineFor(superstructure.idle(), intakeStructure.extend(), intakeStructure.intake()),
                drive.run(drive::stopWithX)
                        .alongWith(superstructure.requestScoring(), intakeStructure.agitate()).withTimeout(5)
        );
    }

    private static Command LEFT_SWIPE_MID(Drive drive, Superstructure superstructure, IntakeStructure intakeStructure, RobotState state) {
        return new SequentialCommandGroup(
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.LEFT_TRENCHLINE_NEUTRAL_FAR, 3.0))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.LEFT_SWIPE_ENDPOINT_MID, 0.75, 0.25))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.LEFT_TRENCHLINE_NEUTRAL_MID, 1.75))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.LEFT_TRENCHLINE_TRENCH, 1.0))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.LEFT_TRENCHLINE_AZ_MID, 1.0))
                        .deadlineFor(superstructure.idle(), intakeStructure.extend(), intakeStructure.intake()),
                drive.run(drive::stopWithX)
                        .alongWith(superstructure.requestScoring(), intakeStructure.agitate()).withTimeout(5)
        );
    }

    private static Command RIGHT_SWIPE_MID(Drive drive, Superstructure superstructure, IntakeStructure intakeStructure, RobotState state) {
        return new SequentialCommandGroup(
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.RIGHT_TRENCHLINE_NEUTRAL_FAR, 3.0))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.RIGHT_SWIPE_ENDPOINT_MID, 0.75, 0.25))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.RIGHT_TRENCHLINE_NEUTRAL_MID, 1.75))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.RIGHT_TRENCHLINE_TRENCH, 1.0))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.RIGHT_TRENCHLINE_AZ_MID, 1.0))
                        .deadlineFor(superstructure.idle(), intakeStructure.extend(), intakeStructure.intake()),
                drive.run(drive::stopWithX)
                        .alongWith(superstructure.requestScoring(), intakeStructure.agitate()).withTimeout(5)
        );
    }

    private static Command RIGHT_SWIPE_FAR(Drive drive, Superstructure superstructure, IntakeStructure intakeStructure, RobotState state) {
        return new SequentialCommandGroup(
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.RIGHT_TRENCHLINE_NEUTRAL_FAR, 2.0))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.RIGHT_SWIPE_ENDPOINT_FAR, 0.75, 0.25))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.RIGHT_TRENCHLINE_NEUTRAL_MID_FAR, 1.75))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.RIGHT_TRENCHLINE_TRENCH, 1.0))
                        .deadlineFor(superstructure.trackHub(), intakeStructure.extend(), intakeStructure.intake()),
                drive.defer(() -> DriveCommands.PIDToPose(drive, state, AutoPoses.RIGHT_TRENCHLINE_AZ_MID, 1.0))
                        .deadlineFor(superstructure.idle(), intakeStructure.extend(), intakeStructure.intake()),
                drive.run(drive::stopWithX)
                        .alongWith(superstructure.requestScoring(), intakeStructure.agitate()).withTimeout(5)
        );
    }
}
