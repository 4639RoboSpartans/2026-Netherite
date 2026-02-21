/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Voltage;
import org.team4639.frc2026.util.PortConfiguration;

import static edu.wpi.first.units.Units.Volts;

public class ShooterIOSparkFlex implements ShooterIO {
    private final SparkFlex leftShooter;
    private final SparkFlex rightShooter;

    private final SparkClosedLoopController closedLoopController;

    public static final ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    public static final SparkFlexConfig shooterConfig = new SparkFlexConfig();
    public static final SparkFlexConfig leaderConfig = new SparkFlexConfig();
    public static final SparkFlexConfig followerConfig = new SparkFlexConfig();

    private final double SHOOTER_GEAR_RATIO = 1.0;

    public ShooterIOSparkFlex(PortConfiguration ports) {
        shooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        shooterConfig.smartCurrentLimit(40);
        shooterConfig.secondaryCurrentLimit(80);

        updateGains();

        shooterConfig.apply(closedLoopConfig);
        shooterConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(5)
                .appliedOutputPeriodMs(5)
                .busVoltagePeriodMs(5)
                .outputCurrentPeriodMs(5);

        shooterConfig.encoder.velocityConversionFactor(1.0 / 60.0); // convert REV native RPM to RPS, better for velocity PID on spark

        leftShooter = new SparkFlex(ports.shooterMotorLeftID.getDeviceNumber(), SparkLowLevel.MotorType.kBrushless);
        rightShooter = new SparkFlex(ports.shooterMotorRightID.getDeviceNumber(), SparkLowLevel.MotorType.kBrushless);

        leaderConfig.apply(shooterConfig);
        followerConfig.apply(shooterConfig);
        followerConfig.follow(leftShooter.getDeviceId());

        leftShooter.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightShooter.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        rightShooter.setVoltage(targetVoltage);
    }

    @Override
    public void setRPM(double targetRPM) {
        double applied = targetRPM * SHOOTER_GEAR_RATIO / 60.0;
        closedLoopController.setSetpoint(applied, SparkBase.ControlType.kVelocity);
    }

    public void updateGains() {
        closedLoopConfig.pid(PIDs.shooterKp.get(), PIDs.shooterKi.get(), PIDs.shooterKd.get());
        closedLoopConfig.apply(new FeedForwardConfig()
                .kS(PIDs.shooterKs.get())
                .kV(PIDs.shooterKv.get())
                .kA(PIDs.shooterKa.get()));
    }

    @Override
    public void applyNewGains() {
        updateGains();
        leaderConfig.apply(closedLoopConfig);
        followerConfig.apply(closedLoopConfig);
        leftShooter.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightShooter.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
