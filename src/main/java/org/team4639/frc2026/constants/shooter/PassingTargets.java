package org.team4639.frc2026.constants.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import lombok.RequiredArgsConstructor;
import org.team4639.frc2026.FieldConstants;

// where to point at when we pass, doesn't really have all that much to do with where the ball lands
@RequiredArgsConstructor
public enum PassingTargets {
    LEFT(FieldConstants.LeftBump.farLeftCorner),
    RIGHT(FieldConstants.RightBump.farRightCorner);

    public final Translation2d target;
}
