/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.subsystems.extension.Extension;
import org.team4639.frc2026.subsystems.intake.Intake;

public class IntakeCommands {
  private static final double AGITATE_PERIOD = 1.0;

  public static Command extend(Extension extension) {
    return extension.dummy.runOnce(() -> extension.setWantedState(Extension.WantedState.EXTENDED));
  }

  public static Command retract(Extension extension) {
    return extension.dummy.runOnce(() -> extension.setWantedState(Extension.WantedState.IDLE));
  }

  public static Command intake(Intake intake) {
    return intake.dummy.runOnce(() -> intake.setWantedState(Intake.WantedState.INTAKE));
  }

  public static Command stop(Intake intake) {
    return intake.dummy.runOnce(() -> intake.setWantedState(Intake.WantedState.IDLE));
  }

  public static Command outtake(Intake intake) {
    return intake.dummy.runOnce(() -> intake.setWantedState(Intake.WantedState.OUTTAKE));
  }

  public static Command agitate(Extension extension, Intake intake) {
    return Commands.either(
        agitateInOut(extension, intake),
        agitateOutIn(extension, intake),
        () ->
            RobotState.getInstance().getExtensionStates().getFirst() == Extension.WantedState.IDLE);
  }

  private static Command agitateInOut(Extension extension, Intake intake) {
    return new ParallelCommandGroup(
        new SequentialCommandGroup(
                retract(extension).andThen(Commands.idle()).withTimeout(AGITATE_PERIOD / 2),
                extend(extension).andThen(Commands.idle()).withTimeout(AGITATE_PERIOD / 2))
            .repeatedly(),
        intake(intake));
  }

  private static Command agitateOutIn(Extension extension, Intake intake) {
    return new ParallelCommandGroup(
        new SequentialCommandGroup(
                extend(extension).andThen(Commands.idle()).withTimeout(AGITATE_PERIOD / 2),
                retract(extension).andThen(Commands.idle()).withTimeout(AGITATE_PERIOD / 2))
            .repeatedly(),
        intake(intake));
  }
}
