/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team4639.frc2026.auto.AutoCommands;
import org.team4639.frc2026.commands.DriveCommands;
import org.team4639.frc2026.commands.IntakeCommands;
import org.team4639.frc2026.commands.LEDCommands;
import org.team4639.frc2026.commands.SuperstructureCommands;
import org.team4639.frc2026.constants.ports.Netherite;
import org.team4639.frc2026.constants.shooter.LookupTables;
import org.team4639.frc2026.subsystems.drive.*;
import org.team4639.frc2026.subsystems.drive.generated.TunerConstants;
import org.team4639.frc2026.subsystems.extension.Extension;
import org.team4639.frc2026.subsystems.extension.IntakeExtensionIO;
import org.team4639.frc2026.subsystems.extension.IntakeExtensionIOSim;
import org.team4639.frc2026.subsystems.extension.IntakeExtensionIOTalonFX;
import org.team4639.frc2026.subsystems.hood.Hood;
import org.team4639.frc2026.subsystems.hood.HoodIO;
import org.team4639.frc2026.subsystems.hood.HoodIOSim;
import org.team4639.frc2026.subsystems.hood.HoodIOTalonFX;
import org.team4639.frc2026.subsystems.intake.*;
import org.team4639.frc2026.subsystems.intake.IntakeRollerIOTalonFX;
import org.team4639.frc2026.subsystems.kicker.Kicker;
import org.team4639.frc2026.subsystems.kicker.KickerIO;
import org.team4639.frc2026.subsystems.kicker.KickerIOTalonFX;
import org.team4639.frc2026.subsystems.ledkicker.LEDKicker;
import org.team4639.frc2026.subsystems.ledkicker.LEDKickerIO;
import org.team4639.frc2026.subsystems.ledkicker.LEDKickerIOHardware;
import org.team4639.frc2026.subsystems.ledkicker.LEDKickerIOSim;
import org.team4639.frc2026.subsystems.shooter.Shooter;
import org.team4639.frc2026.subsystems.shooter.ShooterIO;
import org.team4639.frc2026.subsystems.shooter.ShooterIOSim;
import org.team4639.frc2026.subsystems.shooter.ShooterIOSparkFlex;
import org.team4639.frc2026.subsystems.spindexer.Spindexer;
import org.team4639.frc2026.subsystems.spindexer.SpindexerIO;
import org.team4639.frc2026.subsystems.spindexer.SpindexerIOTalonFX;
import org.team4639.frc2026.subsystems.turret.*;
import org.team4639.frc2026.subsystems.vision.*;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.oi.OI;
import org.team4639.lib.util.Commands2;
import org.team4639.lib.util.LoggedLazyAutoChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final PortConfiguration portConfiguration = Netherite.portConfiguration;

    // Subsystems
    private final Drive drive;

    @SuppressWarnings("unused")
    private final Vision vision;

    private final Intake intake;
    private final Extension extension;
    private final Spindexer spindexer;
    private final Kicker kicker;

    @SuppressWarnings("unused")
    private final TurretCamera turretCamera;

    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;
    private final LEDKicker ledkicker;

    // Controller
    private final CommandXboxController driver = OI.driver;
    private final CommandXboxController operator = OI.operator;

    // Dashboard inputs
    private final LoggedLazyAutoChooser autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        pose -> {});

                intake = new Intake(
                        /* new IntakeRollerIO() {} */
                        new IntakeRollerIOTalonFX(portConfiguration), RobotState.getInstance());

                extension = new Extension(new IntakeExtensionIOTalonFX(portConfiguration), RobotState.getInstance());

                spindexer = new Spindexer(new SpindexerIOTalonFX(portConfiguration), RobotState.getInstance());

                kicker = new Kicker(new KickerIOTalonFX(portConfiguration), RobotState.getInstance());

                turret = new Turret(
                        new TurretIOTalonFX(portConfiguration),
                        new EncoderIOCANCoder(
                                portConfiguration.TurretLeftEncoderID,
                                org.team4639.frc2026.subsystems.turret.Constants.LEFT_ENCODER_OFFSET,
                                org.team4639.frc2026.subsystems.turret.Constants.LEFT_ENCODER_INVERTED),
                        new EncoderIOCANCoder(
                                portConfiguration.TurretRightEncoderID,
                                org.team4639.frc2026.subsystems.turret.Constants.RIGHT_ENCODER_OFFSET,
                                org.team4639.frc2026.subsystems.turret.Constants.RIGHT_ENCODER_INVERTED),
                        RobotState.getInstance());

                hood = new Hood(new HoodIOTalonFX(portConfiguration) /* new HoodIO(){} */, RobotState.getInstance());

                shooter = new Shooter(new ShooterIOSparkFlex(portConfiguration), RobotState.getInstance());

                vision = new Vision(
                        RobotState.getInstance(),
                        new VisionIOLimelight("limelight-left", () -> RobotState.getInstance()
                                .getEstimatedPose()
                                .getRotation()),
                        new VisionIOLimelight("limelight-right", () -> RobotState.getInstance()
                                .getEstimatedPose()
                                .getRotation()));

                turretCamera = new TurretCamera(
                        RobotState.getInstance(),
                        new VisionIOLimelight4(
                                "limelight-turret",
                                () -> RobotState.getInstance().getTurretPose().getRotation()));

                ledkicker = new LEDKicker(new LEDKickerIOHardware(portConfiguration, 150));

                configureButtonBindings();
                break;

            case SIM:
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

                intake = new Intake(new IntakeRollerIOSim(), RobotState.getInstance());

                extension = new Extension(new IntakeExtensionIOSim(), RobotState.getInstance());

                spindexer = new Spindexer(new SpindexerIO() {}, RobotState.getInstance());

                kicker = new Kicker(new KickerIO() {}, RobotState.getInstance());

                turret =
                        new Turret(new TurretIOSim(), new EncoderIOSim(), new EncoderIOSim(), RobotState.getInstance());

                hood = new Hood(new HoodIOSim(), RobotState.getInstance());

                shooter = new Shooter(new ShooterIOSim(), RobotState.getInstance());

                // flip poses so that the vision sees the true on-field pose
                vision = new Vision(
                        RobotState.getInstance(),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera0Name,
                                VisionConstants.robotToCamera0,
                                () -> SimRobot.getInstance()
                                        .getSwerveDriveSimulation()
                                        .getSimulatedDriveTrainPose()),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera1Name,
                                VisionConstants.robotToCamera1,
                                () -> SimRobot.getInstance()
                                        .getSwerveDriveSimulation()
                                        .getSimulatedDriveTrainPose()));

                turretCamera =
                        // new TurretCamera(
                        // RobotState.getInstance(),
                        // new VisionIOPhotonVisionSim(
                        // "Turret-Sim",
                        // new Transform3d(),
                        // () ->
                        // SimRobot.getInstance()
                        // .getSwerveDriveSimulation()
                        // .getSimulatedDriveTrainPose()
                        // .transformBy(
                        // new Transform2d(
                        //
                        // Constants.SimConstants.originToTurretRotation.toTranslation2d(),
                        // Rotation2d.fromRotations(
                        // RobotState.getInstance()
                        // .getScoringState()
                        // .turretRotations())))));
                        new TurretCamera(RobotState.getInstance(), new VisionIO() {});

                ledkicker = new LEDKicker(new LEDKickerIOSim());

                configureSimButtonBindings();
                break;

            default:
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        pose -> {});

                intake = new Intake(new IntakeRollerIO() {}, RobotState.getInstance());

                extension = new Extension(new IntakeExtensionIO() {}, RobotState.getInstance());

                spindexer = new Spindexer(new SpindexerIO() {}, RobotState.getInstance());

                kicker = new Kicker(new KickerIO() {}, RobotState.getInstance());

                turret =
                        new Turret(new TurretIO() {}, new EncoderIO() {}, new EncoderIO() {}, RobotState.getInstance());

                hood = new Hood(new HoodIO() {}, RobotState.getInstance());

                shooter = new Shooter(new ShooterIO() {}, RobotState.getInstance());

                vision = new Vision(RobotState.getInstance());

                turretCamera = new TurretCamera(RobotState.getInstance(), new VisionIO() {});

                ledkicker = new LEDKicker(new LEDKickerIO() {});

                configureButtonBindings();
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedLazyAutoChooser("Auto Choices");

        autoChooser.addOption("OP_LEFT", () -> AutoCommands.OP_LEFT(
                        drive, shooter, hood, turret, spindexer, kicker, extension, intake, RobotState.getInstance())
                .withTimeout(20));

        autoChooser.addOption("OP_RIGHT", () -> AutoCommands.OP_RIGHT(
                        drive, shooter, hood, turret, spindexer, kicker, extension, intake, RobotState.getInstance())
                .withTimeout(20));

        autoChooser.addOption("OP_NEAR_LEFT", () -> AutoCommands.OP_NEAR_LEFT(
                        drive, shooter, hood, turret, spindexer, kicker, extension, intake, RobotState.getInstance())
                .withTimeout(20));

        autoChooser.addOption("OP_NEAR_RIGHT", () -> AutoCommands.OP_NEAR_RIGHT(
                        drive, shooter, hood, turret, spindexer, kicker, extension, intake, RobotState.getInstance())
                .withTimeout(20));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDriveWithX(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> Math.pow(Math.abs(driver.getRightX()), 0.75) * (driver.getRightX() > 0 ? -1 : 1)));

        ledkicker.setDefaultCommand(LEDCommands.useDefaultSchema(ledkicker, RobotState.getInstance()));

        shooter.dummy.setDefaultCommand(
                SuperstructureCommands.idle(shooter, hood, turret, spindexer, kicker, RobotState.getInstance()));

        driver.rightTrigger()
                .whileTrue(SuperstructureCommands.requestScoring(
                        shooter, hood, turret, spindexer, kicker, RobotState.getInstance()));
        driver.leftTrigger()
                .whileTrue(SuperstructureCommands.requestPassing(
                        shooter, hood, turret, spindexer, kicker, RobotState.getInstance()));
        driver.leftBumper().or(driver.rightBumper()).whileTrue(IntakeCommands.agitate(extension, intake));

        driver.x().onTrue(IntakeCommands.extend(extension));
        driver.y().onTrue(IntakeCommands.retract(extension));

        driver.a().onTrue(IntakeCommands.intake(intake));
        driver.b().onTrue(IntakeCommands.stop(intake));

        driver.back().whileTrue(drive.run(drive::autoConfiguration));
        driver.start().onTrue(turret.rezeroAgainstWires());

        driver.povDown().onTrue(IntakeCommands.outtake(intake));

        operator.povUp().onTrue(Commands2.action(() -> LookupTables.RPMFudge = LookupTables.RPMFudge + 0.1));
        operator.povDown().onTrue(Commands2.action(() -> LookupTables.RPMFudge = LookupTables.RPMFudge - 0.1));

        operator.povLeft().whileTrue(turret.overrideCounterClockwise());
        operator.povRight().whileTrue(turret.overrideCounterClockwise());

        operator.x().onTrue(turret.rezeroAgainstWires());

        operator.back()
                .onTrue(Commands2.action(
                        () -> RobotState.getInstance().useTurretBuffer = !RobotState.getInstance().useTurretBuffer));

        operator.y()
                .onTrue(Commands2.action(
                        () -> SuperstructureCommands.turretDisabled = !SuperstructureCommands.turretDisabled));

        operator.a().onTrue(Commands2.action(() -> RobotState.getInstance()
                .setDisableTurretCamera(!RobotState.getInstance().isDisableTurretCamera())));
        operator.b().onTrue(Commands2.action(() -> RobotState.getInstance()
                .setDisableBottomCameras(!RobotState.getInstance().isDisableBottomCameras())));

        operator.leftTrigger()
                .whileTrue(SuperstructureCommands.setClosestOverride(
                        shooter, hood, turret, spindexer, kicker, RobotState.getInstance()));
        operator.leftBumper()
                .whileTrue(SuperstructureCommands.setCloseOverride(
                        shooter, hood, turret, spindexer, kicker, RobotState.getInstance()));
        operator.rightBumper()
                .whileTrue(SuperstructureCommands.setFarOverride(
                        shooter, hood, turret, spindexer, kicker, RobotState.getInstance()));
        operator.rightTrigger()
                .whileTrue(SuperstructureCommands.setFarthestOverride(
                        shooter, hood, turret, spindexer, kicker, RobotState.getInstance()));
    }

    private void configureSimButtonBindings() {
        configureButtonBindings();
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
