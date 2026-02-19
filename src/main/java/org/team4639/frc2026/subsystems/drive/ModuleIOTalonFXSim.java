/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.team4639.lib.util.PhoenixUtil;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
 * Simulation is always based on voltage control.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
    private final SwerveModuleSimulation simulation;

    public ModuleIOTalonFXSim(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
            SwerveModuleSimulation simulation) {
        super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

        this.simulation = simulation;
        simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));

        simulation.useSteerMotorController(
                new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turnTalon, cancoder));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        // Update odometry inputs
        inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

        inputs.odometryDrivePositionsRad = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Radians))
                .toArray();

        inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
    }
}
