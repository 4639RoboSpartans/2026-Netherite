/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.subsystems.drive.Drive;

public class AutoCommands {
  private AutoFactory autoFactory;

  public AutoCommands(Drive driveSubsystem) {
    autoFactory =
        new AutoFactory(
            RobotState.getInstance()::getEstimatedPose,
            RobotState.getInstance()::resetPose,
            driveSubsystem::followTrajectory,
            false,
            driveSubsystem);
  }

  public Command DriverStation_TrenchLine() {
    AutoRoutine routine = autoFactory.newRoutine("DriverStation_TrenchLine");
    AutoTrajectory trajectory = routine.trajectory("DriverStation_TrenchLine");
    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));

    return routine.cmd();
  }

  public Command DriverStation_TrenchLine_DriverStation() {
    AutoRoutine routine = autoFactory.newRoutine("DriverStation_TrenchLine_DriverStation");
    AutoTrajectory trajectory = routine.trajectory("DriverStation_TrenchLine_DriverStation");
    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));

    return routine.cmd();
  }
}
