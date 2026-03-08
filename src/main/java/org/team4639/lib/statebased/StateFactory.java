/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.statebased;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.Supplier;

public class StateFactory {
  public static State instantState(InstantCommand command, Supplier<State> nextState) {
    return new State("INSTANT").withDeadline(command, nextState);
  }

  public static State none() {
    return new State("none");
  }
}
