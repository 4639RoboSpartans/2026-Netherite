/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.util.PortConfiguration;

public class ShooterIOSparkFlex implements ShooterIO {
  private final SparkFlex leftShooter;
  private final SparkFlex rightShooter;

  private final SparkClosedLoopController closedLoopController;

  public static final ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
  public static final SparkFlexConfig shooterConfig = new SparkFlexConfig();
  public static final SparkFlexConfig leaderConfig = new SparkFlexConfig();
  public static final SparkFlexConfig followerConfig = new SparkFlexConfig();

  private final double SHOOTER_GEAR_RATIO = 1.0;

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.074548, 0.10976, 0.044959);
  private final PIDController pid = new PIDController(0.0090597, 0, 0);

  public ShooterIOSparkFlex(PortConfiguration ports) {
    shooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    shooterConfig.smartCurrentLimit(90, 90);

    // updateGains();

    shooterConfig.apply(closedLoopConfig);
    shooterConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(5)
        .appliedOutputPeriodMs(5)
        .busVoltagePeriodMs(5)
        .outputCurrentPeriodMs(5);

    leftShooter =
        new SparkFlex(
            ports.shooterMotorLeftID.getDeviceNumber(), SparkLowLevel.MotorType.kBrushless);
    rightShooter =
        new SparkFlex(
            ports.shooterMotorRightID.getDeviceNumber(), SparkLowLevel.MotorType.kBrushless);

    leaderConfig.apply(shooterConfig);

    leaderConfig
        .idleMode(SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(50, 50)
        .apply(new EncoderConfig().velocityConversionFactor(1 / 60.0));
    leaderConfig.closedLoop.feedForward.sva(0.074548, 0.10976, 0.044959);
    leaderConfig
        .closedLoop
        .p(0.0090597)
        .outputRange(-1, 1)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    followerConfig.apply(shooterConfig);
    followerConfig.follow(leftShooter.getDeviceId(), true);

    leftShooter.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightShooter.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = leftShooter.getClosedLoopController();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.rightConnected = !leftShooter.getFaults().can;
    inputs.leftConnected = !rightShooter.getFaults().can;
    inputs.leftVoltage = leftShooter.getBusVoltage();
    inputs.rightVoltage = rightShooter.getBusVoltage();
    inputs.leftCurrent = leftShooter.getOutputCurrent();
    inputs.rightCurrent = rightShooter.getOutputCurrent();
    inputs.leftTemperature = leftShooter.getMotorTemperature();
    inputs.rightTemperature = rightShooter.getMotorTemperature();
    inputs.leftRPM = (leftShooter.getEncoder().getVelocity() * 60) / SHOOTER_GEAR_RATIO;
    inputs.rightRPM = (rightShooter.getEncoder().getVelocity() * 60) / SHOOTER_GEAR_RATIO;
    inputs.leftRotations = (leftShooter.getEncoder().getPosition() / SHOOTER_GEAR_RATIO);
    inputs.rightRotations = (rightShooter.getEncoder().getPosition() / SHOOTER_GEAR_RATIO);
  }

  @Override
  public void setVoltage(double appliedVolts) {
    Voltage targetVoltage = Volts.of(appliedVolts);
    leftShooter.setVoltage(targetVoltage);
  }

  @Override
  public void setRPM(double targetRPM) {
    double applied = -targetRPM * SHOOTER_GEAR_RATIO / 60.0;
    var err =
        closedLoopController.setSetpoint(
            applied, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    Logger.recordOutput("Shooter Setpoint RPS", closedLoopController.getSetpoint());
    Logger.recordOutput("Shooter RevLib Error", err.toString());
    /*leftShooter.setVoltage(ff.calculate(applied)*/
    /*+ pid.calculate(leftShooter.getEncoder().getVelocity(), applied)*/ ;
  }

  public void updateGains() {
    /*closedLoopConfig.pid(PIDs.shooterKp.get(), PIDs.shooterKi.get(), PIDs.shooterKd.get());
    closedLoopConfig.apply(new FeedForwardConfig()
            .kS(PIDs.shooterKs.get())
            .kV(PIDs.shooterKv.get())
            .kA(PIDs.shooterKa.get()));*/
  }

  @Override
  public void applyNewGains() {
    updateGains();
    leaderConfig.apply(closedLoopConfig);
    followerConfig.apply(closedLoopConfig);
    leftShooter.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightShooter.configure(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
