/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import lombok.Getter;
import lombok.Setter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.Constants.Mode;
import org.team4639.frc2026.constants.shooter.ScoringState;
import org.team4639.frc2026.subsystems.drive.Drive;
import org.team4639.lib.util.VirtualSubsystem;

import static edu.wpi.first.units.Units.*;

public class SimRobot extends VirtualSubsystem {
    @Getter
    private static volatile SimRobot instance = new SimRobot();

    @Setter
    @Getter
    private SwerveDriveSimulation swerveDriveSimulation = null;

    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(Constants.RobotConstants.ROBOT_MASS_KG))
            .withCustomModuleTranslations(Drive.getModuleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(COTS.ofMark4i(
                    DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1), Constants.RobotConstants.WHEEL_COF, 3))
            .withBumperSize(Inches.of(32), Inches.of(32));

    @Override
    public void periodic() {
        // DO nothing
    }

    @Override
    public void periodicAfterScheduler() {
        if (Constants.currentMode == Mode.SIM) { // Only do if it is simulation AND not replay
            SimulatedArena.getInstance().simulationPeriodic();
            Logger.recordOutput("Sim/SimulatedDrivetrainPose", swerveDriveSimulation.getSimulatedDriveTrainPose());
            Logger.recordOutput("Sim/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        }
    }

    public void setupDriveSim() {
        this.swerveDriveSimulation = new SwerveDriveSimulation(mapleSimConfig, new Pose2d(3, 3, Rotation2d.kZero));
        var arena = new Arena2026Rebuilt(false);
        arena.setEfficiencyMode(false);
        SimulatedArena.overrideInstance(arena);
        SimulatedArena.getInstance().addDriveTrainSimulation(this.swerveDriveSimulation);
    }

    public void shootFuel(ScoringState scoringState) {
        RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                // Specify the position of the chassis when the note is launched
                swerveDriveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
                new Translation2d(-Units.inchesToMeters(5.84), 0).rotateBy(swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation()),
                // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
                swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                // The shooter facing direction is the same as the robot’s facing direction
                swerveDriveSimulation.getSimulatedDriveTrainPose().getRotation().plus(Rotation2d.fromRotations(scoringState.turretAngle().in(Rotations))),
                // Initial height of the flying note
                Meters.of(0.508),
                // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
                Meters.per(Second).of(scoringState.shooterRPM().in(Radians.per(Second)) * 0.0508),
                // The angle at which the note is launched
                Rotations.of(0.25).minus(scoringState.hoodAngle())
        );
        fuelOnFly.setHitTargetCallBack(() -> System.out.println("FUEL hits HUB!"));
        SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
    }

    public void resetPose(Pose2d pose) {
        this.swerveDriveSimulation.setSimulationWorldPose(pose);
    }
}
