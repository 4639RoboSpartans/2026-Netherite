/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import org.team4639.frc2026.FieldConstants;

public class PassingTargets {
  public static final Translation2d LEFT =
      new Translation2d(
          FieldConstants.LinesVertical.allianceZone / 3., FieldConstants.fieldWidth * 5. / 6.);

  public static final Translation2d RIGHT =
      new Translation2d(
          FieldConstants.LinesVertical.allianceZone / 3., FieldConstants.fieldWidth * 1. / 6.);
}
