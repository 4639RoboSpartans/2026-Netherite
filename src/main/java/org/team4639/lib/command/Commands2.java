/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import java.util.function.Supplier;

public class Commands2 {

  /**
   * Defers a command
   *
   * @param command supplies a command. This should create a new command every time it runs, but
   *     each command should have the same requirements
   * @return a deferred command
   */
  public static DeferredCommand defer(Supplier<Command> command) {
    var preliminaryCommand = command.get();
    var ret = new DeferredCommand(command, preliminaryCommand.getRequirements());
    preliminaryCommand = null;
    return ret;
  }
}
