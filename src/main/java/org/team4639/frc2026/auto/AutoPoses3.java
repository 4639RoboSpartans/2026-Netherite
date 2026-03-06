/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.team4639.frc2026.FieldConstants;

public class AutoPoses3 {
    public static final Pose2d LBSP = new Pose2d(
            3.443,
            5.602,
            Rotation2d.fromDegrees(-135)
    );

    public static final Pose2d RBSP = mirror(LBSP);

    private static Pose2d mirror(Pose2d leftSidePose) {
        return new Pose2d(
                leftSidePose.getX(),
                FieldConstants.fieldWidth - leftSidePose.getY(),
                leftSidePose.getRotation().times(-1)
        );
    }
}
