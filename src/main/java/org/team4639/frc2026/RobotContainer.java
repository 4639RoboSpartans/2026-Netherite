/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.team4639.frc2026.auto.AutoCommands;
import org.team4639.frc2026.commands.DriveCommands;
import org.team4639.frc2026.commands.LEDCommands;
import org.team4639.frc2026.constants.led.Patterns;
import org.team4639.frc2026.constants.ports.Netherite;
import org.team4639.frc2026.constants.shooter.ScoringState;
import org.team4639.frc2026.subsystems.IntakeStructure;
import org.team4639.frc2026.subsystems.Superstructure;
import org.team4639.frc2026.subsystems.drive.*;
import org.team4639.frc2026.subsystems.drive.generated.TunerConstants;
import org.team4639.frc2026.subsystems.extension.Extension;
import org.team4639.frc2026.subsystems.extension.IntakeExtensionIO;
import org.team4639.frc2026.subsystems.extension.IntakeExtensionIOSim;
import org.team4639.frc2026.subsystems.extension.IntakeExtensionIOTalonFX;
import org.team4639.frc2026.subsystems.intake.*;
import org.team4639.frc2026.subsystems.kicker.Kicker;
import org.team4639.frc2026.subsystems.kicker.KickerIO;
import org.team4639.frc2026.subsystems.kicker.KickerIOTalonFX;
import org.team4639.frc2026.subsystems.ledkicker.LEDKicker;
import org.team4639.frc2026.subsystems.ledkicker.LEDKickerIO;
import org.team4639.frc2026.subsystems.ledkicker.LEDKickerIOHardware;
import org.team4639.frc2026.subsystems.ledkicker.LEDKickerIOSim;
import org.team4639.frc2026.subsystems.spindexer.Spindexer;
import org.team4639.frc2026.subsystems.spindexer.SpindexerIO;
import org.team4639.frc2026.subsystems.spindexer.SpindexerIOTalonFX;
import org.team4639.frc2026.subsystems.hood.Hood;
import org.team4639.frc2026.subsystems.hood.HoodIO;
import org.team4639.frc2026.subsystems.hood.HoodIOSim;
import org.team4639.frc2026.subsystems.hood.HoodIOTalonFX;
import org.team4639.frc2026.subsystems.shooter.Shooter;
import org.team4639.frc2026.subsystems.shooter.ShooterIO;
import org.team4639.frc2026.subsystems.shooter.ShooterIOSim;
import org.team4639.frc2026.subsystems.shooter.ShooterIOSparkFlex;
import org.team4639.frc2026.subsystems.turret.*;
import org.team4639.frc2026.subsystems.vision.*;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.oi.DeadbandXboxController;
import org.team4639.lib.statebased2.StateMachine2;
import org.team4639.lib.util.LoggedLazyAutoChooser;
import org.team4639.lib.util.LoggedTunableNumber;
import org.team4639.lib.util.geometry.AllianceFlipUtil;

import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final PortConfiguration portConfiguration = Netherite.portConfiguration;

    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Intake intake;
    private final Extension extension;
    private final Spindexer spindexer;
    private final Kicker kicker;
    private final TurretCamera turretCamera;
    private final Hood hood;
    private final Shooter shooter;
    private final Turret turret;
    private final LEDKicker ledkicker;

    private final Superstructure superstructure;
    private final IntakeStructure intakeStructure;

    // Controller
    private final CommandXboxController driver = new DeadbandXboxController(0);
    private final CommandXboxController operator = new DeadbandXboxController(1);

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
                        pose -> {}
                );

                intake = new Intake(
                        new IntakeRollerIOTalonFX(portConfiguration),
                        RobotState.getInstance()
                );

                extension = new Extension(
                        new IntakeExtensionIO() {},
                        RobotState.getInstance()
                );

                spindexer = new Spindexer(new SpindexerIOTalonFX(portConfiguration), RobotState.getInstance());

                kicker = new Kicker(new KickerIOTalonFX(portConfiguration), RobotState.getInstance());

                turret = new Turret(
                        new TurretIOTalonFX(portConfiguration),
                        new EncoderIOCANCoder(
                                portConfiguration.TurretLeftEncoderID,
                                org.team4639.frc2026.subsystems.turret.Constants.LEFT_ENCODER_OFFSET,
                                org.team4639.frc2026.subsystems.turret.Constants.LEFT_ENCODER_INVERTED
                        ),
                        new EncoderIOCANCoder(
                                portConfiguration.TurretRightEncoderID,
                                org.team4639.frc2026.subsystems.turret.Constants.RIGHT_ENCODER_OFFSET,
                                org.team4639.frc2026.subsystems.turret.Constants.RIGHT_ENCODER_INVERTED
                        ),
                        RobotState.getInstance()
                );

                hood = new Hood(new HoodIOTalonFX(portConfiguration), RobotState.getInstance());

                shooter = new Shooter(new ShooterIOSparkFlex(portConfiguration), RobotState.getInstance());

                vision = new Vision(
                        RobotState.getInstance(),
                        new VisionIOLimelight("limelight-left", () -> RobotState.getInstance().getEstimatedPose().getRotation()),
                        new VisionIOLimelight("limelight-right", () -> RobotState.getInstance().getEstimatedPose().getRotation())
                );

                turretCamera = new TurretCamera(RobotState.getInstance(), new VisionIOLimelight4("limelight-turret", () -> RobotState.getInstance().getTurretPose().getRotation()));

                ledkicker = new LEDKicker(new LEDKickerIOHardware(portConfiguration, 150));

                superstructure = new Superstructure(turret, hood, shooter, kicker, spindexer, RobotState.getInstance());

                intakeStructure = new IntakeStructure(intake, extension, RobotState.getInstance());

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
                        SimRobot.getInstance()::resetPose
                );

                intake = new Intake(
                        new IntakeRollerIOSim(),
                        RobotState.getInstance()
                );

                extension = new Extension(
                        new IntakeExtensionIOSim(),
                        RobotState.getInstance()
                );

                spindexer = new Spindexer(new SpindexerIO() {}, RobotState.getInstance());

                kicker = new Kicker(new KickerIO() {}, RobotState.getInstance());

                turret = new Turret(
                        new TurretIOSim(),
                        new EncoderIOSim(),
                        new EncoderIOSim(),
                        RobotState.getInstance()
                );

                hood = new Hood(new HoodIOSim(), RobotState.getInstance());

                shooter = new Shooter(new ShooterIOSim(), RobotState.getInstance());

                // flip poses so that the vision sees the true on-field pose
                vision = new Vision(
                        RobotState.getInstance()//,
//                        new VisionIOPhotonVisionSim(
//                                VisionConstants.camera0Name,
//                                VisionConstants.robotToCamera0,
//                                () -> AllianceFlipUtil.apply(SimRobot.getInstance()
//                                        .getSwerveDriveSimulation()
//                                        .getSimulatedDriveTrainPose())),
//                        new VisionIOPhotonVisionSim(
//                                VisionConstants.camera1Name,
//                                VisionConstants.robotToCamera1,
//                                () -> AllianceFlipUtil.apply(SimRobot.getInstance()
//                                        .getSwerveDriveSimulation()
//                                        .getSimulatedDriveTrainPose()))
                );

//                turretCamera = new TurretCamera(RobotState.getInstance(), new VisionIOPhotonVisionSim("Turret-Sim", new Transform3d(), () -> RobotState.getInstance().getTurretPose()));
                turretCamera = new TurretCamera(RobotState.getInstance(), new VisionIO() {});

                ledkicker = new LEDKicker(new LEDKickerIOSim());

                superstructure = new Superstructure(turret, hood, shooter, kicker, spindexer, RobotState.getInstance());

                intakeStructure = new IntakeStructure(intake, extension, RobotState.getInstance());

                configureSimButtonBindings();
                break;

            default:
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        pose -> {}
                );

                intake = new Intake(
                        new IntakeRollerIO() {},
                        RobotState.getInstance()
                );

                extension = new Extension(
                        new IntakeExtensionIO() {},
                        RobotState.getInstance()
                );

                spindexer = new Spindexer(new SpindexerIO() {}, RobotState.getInstance());

                kicker = new Kicker(new KickerIO() {}, RobotState.getInstance());

                turret = new Turret(
                        new TurretIO() {},
                        new EncoderIO() {},
                        new EncoderIO() {},
                        RobotState.getInstance()
                );

                hood = new Hood(new HoodIO() {}, RobotState.getInstance());

                shooter = new Shooter(new ShooterIO() {}, RobotState.getInstance());

                vision = new Vision(RobotState.getInstance());

                turretCamera = new TurretCamera(RobotState.getInstance(), new VisionIO() {});

                ledkicker = new LEDKicker(new LEDKickerIO() {});

                superstructure = new Superstructure(turret, hood, shooter, kicker, spindexer, RobotState.getInstance());

                intakeStructure = new IntakeStructure(intake, extension, RobotState.getInstance());

                configureButtonBindings();
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
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDriveWithX(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

        ledkicker.setDefaultCommand(LEDCommands.useDefaultSchema(ledkicker, RobotState.getInstance()));

        superstructure.setDefaultCommand(superstructure.idle());
        driver.rightTrigger().whileTrue(superstructure.requestScoring());
        driver.leftTrigger().whileTrue(superstructure.requestPassing());

        driver.a().onTrue(intakeStructure.intake());
        driver.b().onTrue(intakeStructure.stopIntake());

        driver.x().onTrue(intakeStructure.extend());
        driver.y().onTrue(intakeStructure.retract());

        driver.rightBumper().or(driver.leftBumper()).whileTrue(intakeStructure.agitate());
    }

    private void configureSimButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDriveWithX(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

        driver.rightTrigger().whileTrue(
                Commands.parallel(
                        DriveCommands.joystickDriveWithX(
                                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()
                        ),
                        Commands.run(
                                () -> {
                                    ScoringState currentScoringState = RobotState.getInstance().getScoringState();
                                    turret.setWantedState(Turret.WantedState.SCORING, RobotState.getInstance().calculateClosestDriveAndTurretRotation(RobotState.getInstance().calculateScoringState())[1].getRotations(), 0);
                                    shooter.setWantedState(Shooter.WantedState.SCORING, currentScoringState.shooterRPM().in(Rotations.per(Minute)));
                                    hood.setWantedState(Hood.WantedState.SCORING, currentScoringState.hoodAngle().in(Rotations));
                                }
                        )
                )
        );

        driver.leftTrigger().whileTrue(
                Commands.run(
                        () -> intake.setWantedState(Intake.WantedState.INTAKE)
                )
        );

        driver.leftBumper().onTrue(
                Commands.run(
                        () -> extension.setWantedState(Extension.WantedState.EXTENDED)
                )
        );

        driver.rightBumper().onTrue(
                Commands.run(
                        () -> extension.setWantedState(Extension.WantedState.IDLE)
                )
        );
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
