/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.RequiredArgsConstructor;
import org.team4639.frc2026.FieldConstants;

// where to point at when we pass, doesn't really have all that much to do with where the ball lands
@RequiredArgsConstructor
public enum PassingTargets {
    LEFT(new Translation2d(0, 5)),
    RIGHT(new Translation2d(0, FieldConstants.fieldWidth-5));

    public final Translation2d target;
}
