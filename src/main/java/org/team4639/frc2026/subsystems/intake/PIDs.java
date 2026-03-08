/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.intake;

import org.team4639.lib.util.LoggedTunableNumber;

public class PIDs {
  public static final LoggedTunableNumber rollerkP =
      new LoggedTunableNumber("Roller/kP").initDefault(0);
  public static final LoggedTunableNumber rollerkI =
      new LoggedTunableNumber("Roller/kI").initDefault(0);
  public static final LoggedTunableNumber rollerkD =
      new LoggedTunableNumber("Roller/kD").initDefault(0);
  public static final LoggedTunableNumber rollerkS =
      new LoggedTunableNumber("Roller/kS").initDefault(0);
  public static final LoggedTunableNumber rollerkV =
      new LoggedTunableNumber("Roller/kV").initDefault(0);
  public static final LoggedTunableNumber rollerkA =
      new LoggedTunableNumber("Roller/kA").initDefault(0);
}
