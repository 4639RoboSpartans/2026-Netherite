/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.auto.AutoCommands;
import org.team4639.frc2026.commands.DriveCommands;
import org.team4639.frc2026.constants.ports.Netherite;
import org.team4639.frc2026.subsystems.drive.*;
import org.team4639.frc2026.subsystems.drive.generated.TunerConstants;
import org.team4639.frc2026.subsystems.intake.*;
import org.team4639.frc2026.constants.ports.Netherite;
import org.team4639.frc2026.subsystems.Superstructure;
import org.team4639.frc2026.subsystems.drive.*;
import org.team4639.frc2026.subsystems.drive.generated.TunerConstants;
import org.team4639.frc2026.subsystems.kicker.Kicker;
import org.team4639.frc2026.subsystems.kicker.KickerIO;
import org.team4639.frc2026.subsystems.kicker.KickerIOTalonFX;
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
import org.team4639.frc2026.subsystems.vision.Vision;
import org.team4639.frc2026.subsystems.vision.VisionConstants;
import org.team4639.frc2026.subsystems.vision.VisionIOPhotonVisionSim;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.statebased2.StateMachine2;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.LoggedLazyAutoChooser;
import org.team4639.lib.util.geometry.AllianceFlipUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final PortConfiguration portConfiguration = Netherite.portConfiguration;

    private final PortConfiguration portConfiguration = Netherite.portConfiguration;

    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Kicker kicker;
    private final Hood hood;
   // private final Shooter shooter;
    private final Turret turret;
    //private final Superstructure superstructure;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedLazyAutoChooser autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
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
                        pose -> {}
                );

                // No cameras on real robot yet
                vision = new Vision(RobotState.getInstance());

                hood = new Hood(new HoodIOTalonFX(portConfiguration), RobotState.getInstance());
                //shooter = new Shooter(new ShooterIOSparkFlex(portConfiguration), RobotState.getInstance());
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

                //superstructure = new Superstructure(shooter, turret, hood, RobotState.getInstance());

                configureButtonBindings();

                intake = new Intake(
                        new IntakeExtensionIOTalonFX(portConfiguration),
                        new IntakeRollerIOTalonFX(portConfiguration),
                        RobotState.getInstance()
                );

                // Configure the button bindings
                configureButtonBindings();
                spindexer = new Spindexer(new SpindexerIOTalonFX(portConfiguration), RobotState.getInstance());

                kicker = new Kicker(new KickerIOTalonFX(portConfiguration), RobotState.getInstance());

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

                hood = new Hood(new HoodIOSim(), RobotState.getInstance());
                //shooter = new Shooter(new ShooterIOSim(), RobotState.getInstance());
                turret = new Turret(
                        new TurretIOSim(),
                        new EncoderIOSim(),
                        new EncoderIOSim(),
                        RobotState.getInstance()
                );

                //superstructure = new Superstructure(shooter, turret, hood, RobotState.getInstance());

                configureSimButtonBindings();


                intake = new Intake(
                        new IntakeExtensionIOSim(),
                        new IntakeRollerIOSim(),
                        RobotState.getInstance()
                );

                // Configure the button bindings
                configureSimButtonBindings();
                spindexer = new Spindexer(new SpindexerIO() {}, RobotState.getInstance());

                kicker = new Kicker(new KickerIO() {}, RobotState.getInstance());

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        pose -> {
                        });

                vision = new Vision(RobotState.getInstance());

                hood = new Hood(new HoodIO() {}, RobotState.getInstance());
                //shooter = new Shooter(new ShooterIO() {}, RobotState.getInstance());
                turret = new Turret(
                        new TurretIO() {},
                        new EncoderIO() {},
                        new EncoderIO() {},
                        RobotState.getInstance()
                );

                //superstructure = new Superstructure(shooter, turret, hood, RobotState.getInstance());


                intake = new Intake(
                        new IntakeExtensionIO() {},
                        new IntakeRollerIO() {},
                        RobotState.getInstance()
                );
                spindexer = new Spindexer(new SpindexerIO() {}, RobotState.getInstance());

                kicker = new Kicker(new KickerIO() {}, RobotState.getInstance());
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
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        //controller.a().whileTrue(DriveCommands.joystickDriveAtAngle(drive, () -> 1, () -> 0, () -> Rotation2d.kZero));

        controller.x().whileTrue(turret.getSysID().getRoutine().quasistatic(SysIdRoutine.Direction.kForward));
        controller.y().whileTrue(turret.getSysID().getRoutine().quasistatic(SysIdRoutine.Direction.kReverse));
        controller.a().whileTrue(turret.getSysID().getRoutine().dynamic(SysIdRoutine.Direction.kForward));
        controller.b().whileTrue(turret.getSysID().getRoutine().dynamic(SysIdRoutine.Direction.kReverse));
    }

    private void configureSimButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));
        // Default command, normal field-relative drive
        StateMachine2 indexing = new StateMachine2().activeDuring(StateMachine2.ActiveMode.TELEOP).publishToNT("IndexingStates");
        indexing.template(state -> {
            state.onExit(
                    new InstantCommand(() -> kicker.setWantedState(Kicker.WantedState.IDLE)),
                    new InstantCommand(() -> spindexer.setWantedState(Spindexer.WantedState.IDLE))
            );
            return state;
        });
        var INDEXING_OFF = indexing.defaultState("OFF")
                .onEnter(
                        new InstantCommand(() -> kicker.setWantedState(Kicker.WantedState.IDLE)),
                        new InstantCommand(() -> spindexer.setWantedState(Spindexer.WantedState.IDLE))
                );

        var INDEXING_ON = indexing.state("ON")
                .onEnter(
                        new InstantCommand(() -> kicker.setWantedState(Kicker.WantedState.KICK)),
                        new InstantCommand(() -> spindexer.setWantedState(Spindexer.WantedState.SPIN))
                );

        INDEXING_ON.onTrigger(controller.x(), () -> INDEXING_OFF);
        INDEXING_OFF.onTrigger(controller.x(), () -> INDEXING_ON);

        StateMachine2 sintake = new StateMachine2().activeDuring(StateMachine2.ActiveMode.TELEOP).publishToNT("Intake States");

        var INTAKE_IN = sintake.defaultState("IN")
                .onEnter(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.IDLE)));
        var INTAKE_OUT = sintake.state("OUT")
                .onEnter(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.INTAKE)));

        INTAKE_IN.onTrigger(controller.a(), () -> INTAKE_OUT);
        INTAKE_OUT.onTrigger(controller.a(), () -> INTAKE_IN);

        var INTAKE_AGITATE = sintake.state("AGITATE")
                .onEnter(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.IDLE)))
                .withDeadline(Commands.waitSeconds(1.0), () -> INTAKE_OUT);

        INTAKE_OUT.onTrigger(controller.b(), () -> INTAKE_AGITATE);

        /*intake.setWantedState(Intake.WantedState.INTAKE);

        controller.x().whileTrue(intake.getRollerSysID().getRoutine().quasistatic(SysIdRoutine.Direction.kForward));
        controller.y().whileTrue(intake.getRollerSysID().getRoutine().quasistatic(SysIdRoutine.Direction.kReverse));
        controller.a().whileTrue(intake.getRollerSysID().getRoutine().dynamic(SysIdRoutine.Direction.kForward));
        controller.b().whileTrue(intake.getRollerSysID().getRoutine().dynamic(SysIdRoutine.Direction.kReverse));*/
    }

    private void configureSimButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));
        controller.a().onTrue(Commands.runOnce(() -> intake.setWantedState(Intake.WantedState.INTAKE)));
        controller.b().onTrue(Commands.runOnce(() -> intake.setWantedState(Intake.WantedState.IDLE)));
        controller.x().onTrue(Commands.runOnce(
                () -> {
                    spindexer.setWantedState(Spindexer.WantedState.SPIN);
                    kicker.setWantedState(Kicker.WantedState.KICK);
                }
        ));
        controller.x().onFalse(Commands.runOnce(
                () -> {
                    spindexer.setWantedState(Spindexer.WantedState.IDLE);
                    kicker.setWantedState(Kicker.WantedState.IDLE);
                }
        ));

        /*controller.x().whileTrue(kicker.getSysID().getRoutine().quasistatic(SysIdRoutine.Direction.kForward));

        controller.y().whileTrue(kicker.getSysID().getRoutine().quasistatic(SysIdRoutine.Direction.kReverse));
        controller.a().whileTrue(kicker.getSysID().getRoutine().dynamic(SysIdRoutine.Direction.kForward));

        controller.b().whileTrue(kicker.getSysID().getRoutine().dynamic(SysIdRoutine.Direction.kReverse));*/

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void publishComponentPoses() {
        Logger.recordOutput("ZeroedComponentPoses", RobotState.getInstance().getComponentPoses());
    }
}
