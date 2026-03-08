/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import org.team4639.lib.util.LoggedTunableNumber;

public class PIDs {
  public static final LoggedTunableNumber hoodKp =
      new LoggedTunableNumber("Hood/kP").initDefault(0);
  public static final LoggedTunableNumber hoodKi =
      new LoggedTunableNumber("Hood/kI").initDefault(0);
  public static final LoggedTunableNumber hoodKd =
      new LoggedTunableNumber("Hood/kD").initDefault(0);
  public static final LoggedTunableNumber hoodKs =
      new LoggedTunableNumber("Hood/kS").initDefault(0);
  public static final LoggedTunableNumber hoodKv =
      new LoggedTunableNumber("Hood/kV").initDefault(0);
  public static final LoggedTunableNumber hoodKa =
      new LoggedTunableNumber("Hood/kA").initDefault(0);

  public static final LoggedTunableNumber hoodKpSim =
      new LoggedTunableNumber("Hood/kPSim").initDefault(0.02);
  public static final LoggedTunableNumber hoodKiSim =
      new LoggedTunableNumber("Hood/kISim").initDefault(0);
  public static final LoggedTunableNumber hoodKdSim =
      new LoggedTunableNumber("Hood/kDSim").initDefault(0);
}
