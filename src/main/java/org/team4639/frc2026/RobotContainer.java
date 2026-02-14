/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.team4639.frc2026.auto.AutoCommands;
import org.team4639.frc2026.commands.DriveCommands;
import org.team4639.frc2026.subsystems.drive.Drive;
import org.team4639.frc2026.subsystems.drive.GyroIO;
import org.team4639.frc2026.subsystems.drive.GyroIOPigeon2;
import org.team4639.frc2026.subsystems.drive.GyroIOSim;
import org.team4639.frc2026.subsystems.drive.ModuleIO;
import org.team4639.frc2026.subsystems.drive.ModuleIOTalonFX;
import org.team4639.frc2026.subsystems.drive.ModuleIOTalonFXSim;
import org.team4639.frc2026.subsystems.drive.generated.TunerConstants;
import org.team4639.frc2026.subsystems.vision.Vision;
import org.team4639.frc2026.subsystems.vision.VisionConstants;
import org.team4639.frc2026.subsystems.vision.VisionIOPhotonVisionSim;
import org.team4639.lib.util.LoggedLazyAutoChooser;
import org.team4639.lib.util.geometry.AllianceFlipUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedLazyAutoChooser autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        pose -> {});

                // No cameras on real robot yet
                vision = new Vision(RobotState.getInstance());

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                SimRobot.getInstance().setupDriveSim();

                drive = new Drive(
                        new GyroIOSim(SimRobot.getInstance()
                                .getSwerveDriveSimulation()
                                .getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontLeft,
                                SimRobot.getInstance()
                                        .getSwerveDriveSimulation()
                                        .getModules()[0]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.FrontRight,
                                SimRobot.getInstance()
                                        .getSwerveDriveSimulation()
                                        .getModules()[1]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackLeft,
                                SimRobot.getInstance()
                                        .getSwerveDriveSimulation()
                                        .getModules()[2]),
                        new ModuleIOTalonFXSim(
                                TunerConstants.BackRight,
                                SimRobot.getInstance()
                                        .getSwerveDriveSimulation()
                                        .getModules()[3]),
                        SimRobot.getInstance()::resetPose);
                // flip poses so that the vision sees the true on-field pose
                vision = new Vision(
                        RobotState.getInstance(),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera0Name,
                                VisionConstants.robotToCamera0,
                                () -> AllianceFlipUtil.apply(SimRobot.getInstance()
                                        .getSwerveDriveSimulation()
                                        .getSimulatedDriveTrainPose())),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera1Name,
                                VisionConstants.robotToCamera1,
                                () -> AllianceFlipUtil.apply(SimRobot.getInstance()
                                        .getSwerveDriveSimulation()
                                        .getSimulatedDriveTrainPose())));
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        pose -> {});

                vision = new Vision(RobotState.getInstance());
                break;
        }

        // Set up auto routines

        AutoCommands autoCommands = new AutoCommands(drive);

        autoChooser = new LoggedLazyAutoChooser("Auto Choices");
        autoChooser.addOption("DriverStation-TrenchLine", autoCommands::DriverStation_TrenchLine);
        autoChooser.addOption(
                "DriverStation_TrenchLine-DriverStation", autoCommands::DriverStation_TrenchLine_DriverStation);

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", () -> DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", () -> DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        controller.a().whileTrue(DriveCommands.joystickDriveAtAngle(drive, () -> 1, () -> 0, () -> Rotation2d.kZero));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
