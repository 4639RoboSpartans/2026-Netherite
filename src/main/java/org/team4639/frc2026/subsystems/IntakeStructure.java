/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.subsystems.extension.Extension;
import org.team4639.frc2026.subsystems.intake.Intake;

public class IntakeStructure {
    public static final double AGITATE_PERIOD = 1.6;
    private final Intake intake;
    private final Extension extension;

    private final Subsystem intakeDummy;
    private final Subsystem extensionDummy;

    private final RobotState robotState;

    // When the driver commands a state we remember it after using a command such as agitate that
    // changes the state automatically and set it back to the proper state;
    private Extension.WantedState lastExtensionWantedState = Extension.WantedState.IDLE;
    private Intake.WantedState lastIntakeWantedState = Intake.WantedState.IDLE;

    public IntakeStructure(Intake intake, Extension extension, RobotState robotState){
        this.intake = intake;
        this.extension = extension;

        this.intakeDummy = new Subsystem() { };
        this.extensionDummy = new Subsystem() { };

        this.robotState = robotState;

        new Trigger(() -> intakeDummy.getCurrentCommand() == null && lastIntakeWantedState == Intake.WantedState.IDLE).onTrue(stopIntake());
        new Trigger(() -> intakeDummy.getCurrentCommand() == null && lastIntakeWantedState == Intake.WantedState.INTAKE).onTrue(intake());
        new Trigger(() -> intakeDummy.getCurrentCommand() == null && lastIntakeWantedState == Intake.WantedState.OUTTAKE).onTrue(outtake());

        new Trigger(() -> extensionDummy.getCurrentCommand() == null && lastExtensionWantedState == Extension.WantedState.IDLE).onTrue(retract());
        new Trigger(() -> extensionDummy.getCurrentCommand() == null && lastExtensionWantedState == Extension.WantedState.EXTENDED).onTrue(extend());
    }

    public Command extend() {
        return extensionDummy.run(
                () -> {
                    extension.setWantedState(Extension.WantedState.EXTENDED);
                }
        ).finallyDo(this::updateLastWantedStates);
    }

    public Command retract() {
        return Commands.run(
                () -> {
                    extension.setWantedState(Extension.WantedState.IDLE);
                    intake.setWantedState(Intake.WantedState.IDLE);
                }
        , extensionDummy, intakeDummy).finallyDo(this::updateLastWantedStates);
    }

    public Command intake() {
        return intakeDummy.run(
                () -> {
                    intake.setWantedState(Intake.WantedState.INTAKE);
                }
        ).finallyDo(this::updateLastWantedStates);
    }

    public Command stopIntake() {
        return intakeDummy.run(
                () -> {
                    intake.setWantedState(Intake.WantedState.IDLE);
                }
        ).finallyDo(this::updateLastWantedStates);
    }

    public Command outtake() {
        return intakeDummy.run(
                () -> {
                    intake.setWantedState(Intake.WantedState.OUTTAKE);
                }
        ).finallyDo(this::updateLastWantedStates);
    }

    public Command agitate() {
        return Commands.either(agitateInOut(), agitateOutIn(),
                () -> robotState.getExtensionStates().getFirst() == Extension.WantedState.EXTENDED
        );
    }

    private Command agitateInOut() {
        return ((
                retract().withTimeout(AGITATE_PERIOD / 2)
                        .andThen(extend().withTimeout(AGITATE_PERIOD / 2)))
                .repeatedly())
                .alongWith(stopIntake());
    }

    private Command agitateOutIn() {
        return ((
                extend().withTimeout(AGITATE_PERIOD / 2)
                        .andThen(retract().withTimeout(AGITATE_PERIOD / 2)))
                .repeatedly())
                .alongWith(stopIntake());
    }

    private void updateLastWantedStates() {
        this.lastExtensionWantedState = robotState.getExtensionStates().getFirst();
        this.lastIntakeWantedState = robotState.getIntakeStates().getFirst();
    }
}
