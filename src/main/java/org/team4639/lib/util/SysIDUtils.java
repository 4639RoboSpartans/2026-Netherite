/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Function;

public class SysIDUtils {
    public enum ButtonConfiguration {
        XYAB,
        POV_UP_RIGHT_DOWN_LEFT
    }

    public static void bind(
            CommandXboxController controller,
            ButtonConfiguration config,
            Command quasistaticForward,
            Command quasistaticReverse,
            Command dynamicForward,
            Command dynamicReverse) {
        switch (config) {
            case XYAB:
                controller.x().whileTrue(quasistaticForward);
                controller.y().whileTrue(quasistaticReverse);
                controller.a().whileTrue(dynamicForward);
                controller.b().whileTrue(dynamicReverse);
                break;
            case POV_UP_RIGHT_DOWN_LEFT:
                controller.povUp().whileTrue(quasistaticForward);
                controller.povRight().whileTrue(quasistaticReverse);
                controller.povDown().whileTrue(dynamicForward);
                controller.povLeft().whileTrue(dynamicReverse);
                break;
        }
    }

    public static void bind(
            CommandXboxController controller,
            ButtonConfiguration config,
            Function<SysIdRoutine.Direction, Command> quasistatic,
            Function<SysIdRoutine.Direction, Command> dynamic) {
        bind(
                controller,
                config,
                quasistatic.apply(SysIdRoutine.Direction.kForward),
                quasistatic.apply(SysIdRoutine.Direction.kReverse),
                dynamic.apply(SysIdRoutine.Direction.kForward),
                dynamic.apply(SysIdRoutine.Direction.kReverse));
    }

    public static void bind(CommandXboxController controller, ButtonConfiguration config, SysIdRoutine routine) {
        bind(controller, config, routine::quasistatic, routine::dynamic);
    }
}
