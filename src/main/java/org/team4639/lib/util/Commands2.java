/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Commands2 {
    public static Command action(Runnable action) {
        return new InstantCommand(action).ignoringDisable(true);
    }
}
