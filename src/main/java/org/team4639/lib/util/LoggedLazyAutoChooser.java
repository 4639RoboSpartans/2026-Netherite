/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.util;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Auto Chooser intended to mimic the functionality of {@link choreo.auto.AutoChooser} while staying
 * consistent with AdvantageKit logging.
 */
public class LoggedLazyAutoChooser extends VirtualSubsystem {
  private Command command = null;
  private final LoggedDashboardChooser<Supplier<Command>> chooser;
  private final Supplier<Command> DEFAULT_NONE = () -> Commands.none();
  private Supplier<Command> lastState;
  private Supplier<Command> currentState;
  private String key;

  @Getter
  @Accessors(fluent = true)
  /**
   * Verify that the command has changed at least once since the start of the robot program and is
   * not the default option (Commands.none()).
   */
  private boolean hasCommand = false;

  public LoggedLazyAutoChooser(String key) {
    this.key = key;
    this.chooser = new LoggedDashboardChooser<>(key);

    this.lastState = DEFAULT_NONE;
    this.currentState = DEFAULT_NONE;

    chooser.addDefaultOption("NONE", DEFAULT_NONE);
    this.command = DEFAULT_NONE.get();
  }

  public void addOption(String key, Supplier<Command> value) {
    chooser.addOption(key, value);
  }

  @Override
  public void periodic() {
    // DO nothing
  }

  @Override
  public void periodicAfterScheduler() {
    if (RobotState.isDisabled()) {
      this.lastState = this.currentState;
      this.currentState = chooser.get();

      if (this.currentState != null && this.lastState != this.currentState) {
        var tempCommand = this.currentState.get();
        if (tempCommand != null) {
          this.command = tempCommand;
          hasCommand = this.currentState != DEFAULT_NONE;
        }
      }
    }

    Logger.recordOutput(key + "/HasCommand", this.hasCommand());
  }

  public Command get() {
    return command;
  }
}
