/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.subsystems.drive.Drive;
import org.team4639.lib.util.LoggedTunableNumber;

/**
 * only written for going through the trench during auto and should not be trusted for any other
 * purpose
 */
public class CosineAlignToY extends Command {
  private final Drive drive;
  private final Pose2d setpoint;
  private final double xSpeedMetersPerSecond;

  public static LoggedTunableNumber YVelocity =
      new LoggedTunableNumber("CosineAlign/YVelocity").initDefault(5);
  public static LoggedTunableNumber YAcceleration =
      new LoggedTunableNumber("CosineAlign/YAcceleration").initDefault(6);
  public static LoggedTunableNumber YkP = new LoggedTunableNumber("CosineAlign/YkP").initDefault(2);
  public static LoggedTunableNumber HeadingKp =
      new LoggedTunableNumber("CosineAlign/HeadingKp").initDefault(2);

  /* public static LoggedTunableNumber cosPower =
  new LoggedTunableNumber("CosineAlign/CosinePower").initDefault(25);*/

  public static ProfiledPIDController YController =
      new ProfiledPIDController(
          YkP.get(), 0, 0, new TrapezoidProfile.Constraints(YVelocity.get(), YAcceleration.get()));
  public static PIDController headingController = new PIDController(HeadingKp.get(), 0, 0);
  public double cosinePower;

  public CosineAlignToY(
      Drive drive, Pose2d setpoint, double xSpeedMetersPerSecond, double cosinePower) {
    this.drive = drive;
    this.setpoint = setpoint;
    this.xSpeedMetersPerSecond = xSpeedMetersPerSecond;
    this.cosinePower = cosinePower;

    addRequirements(drive);

    Logger.recordOutput("Cosine Align Setpoint", setpoint);
    Logger.recordOutput("Using Cosine Align", false);
  }

  @Override
  public void initialize() {
    YController.setPID(YkP.get(), 0, 0);
    YController.setConstraints(
        new TrapezoidProfile.Constraints(YVelocity.get(), YAcceleration.get()));
    headingController.setPID(HeadingKp.get(), 0, 0);
    headingController.enableContinuousInput(0, 1);

    YController.reset(
        RobotState.getInstance().getEstimatedPose().getY(),
        RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond);
    YController.setGoal(setpoint.getY());
    YController.setTolerance(0);
    headingController.setSetpoint(setpoint.getRotation().getRotations());
    Logger.recordOutput("Using Cosine Align", true);
  }

  @Override
  public void execute() {
    double ySpeed = YController.calculate(RobotState.getInstance().getEstimatedPose().getY());
    ySpeed += YController.getSetpoint().velocity;

    Logger.recordOutput("Cosine Align Y Speed", ySpeed);
    Logger.recordOutput(
        "Cosine Align Setpoint",
        new Pose2d(setpoint.getX(), YController.getSetpoint().position, setpoint.getRotation()));

    double xSpeed =
        Math.abs(
                Math.pow(
                    setpoint
                        .getTranslation()
                        .minus(RobotState.getInstance().getEstimatedPose().getTranslation())
                        .getAngle()
                        .getCos(),
                    cosinePower))
            * Math.signum(-RobotState.getInstance().getEstimatedPose().getX() + setpoint.getX())
            * xSpeedMetersPerSecond;

    Logger.recordOutput("Cosine Align X Speed", xSpeed);

    double omega =
        headingController.calculate(
            RobotState.getInstance().getEstimatedPose().getRotation().getRotations());

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            Units.rotationsToRadians(omega),
            RobotState.getInstance().getEstimatedPose().getRotation());

    drive.runVelocity(speeds);
    Logger.recordOutput("Using Cosine Align", true);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(RobotState.getInstance().getEstimatedPose().getX() - setpoint.getX()) < 0.2;
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("Using Cosine Align", false);
  }
}
