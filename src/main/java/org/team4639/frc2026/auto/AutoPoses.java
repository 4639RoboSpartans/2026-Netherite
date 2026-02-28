/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.team4639.frc2026.FieldConstants;

public class AutoPoses {

    public static final Pose2d LEFT_START = new Pose2d(
            4.372499942779541,
            0.4993302524089813,
            Rotation2d.kZero
    );

    public static final Pose2d RIGHT_START = mirror(LEFT_START);

    public static final Pose2d LEFT_TRENCHLINE_NEUTRAL_FAR = new Pose2d(
            8.670371055603027,
            0.4993302524089813,
            Rotation2d.kZero
    );

    public static final Pose2d LEFT_TRENCHLINE_NEUTRAL_MID_FAR = new Pose2d(
            7.670371055603027,
            0.4993302524089813,
            Rotation2d.kZero
    );

    public static final Pose2d RIGHT_TRENCHLINE_NEUTRAL_FAR = mirror(LEFT_TRENCHLINE_NEUTRAL_FAR);

    public static final Pose2d RIGHT_TRENCHLINE_NEUTRAL_MID_FAR = mirror(LEFT_TRENCHLINE_NEUTRAL_MID_FAR);

    public static final Pose2d LEFT_TRENCHLINE_AZ_FAR = new Pose2d(
            0.5929802656173706,
            0.4993302524089813,
            Rotation2d.kZero
    );

    public static final Pose2d RIGHT_TRENCHLINE_AZ_FAR = mirror(LEFT_TRENCHLINE_AZ_FAR);

    public static final Pose2d LEFT_TRENCHLINE_NEUTRAL_MID = new Pose2d(
            6.484410285949707,
            0.4993302524089813,
            Rotation2d.kZero
    );

    public static final Pose2d RIGHT_TRENCHLINE_NEUTRAL_MID = mirror(LEFT_TRENCHLINE_NEUTRAL_MID);

    public static final Pose2d LEFT_TRENCHLINE_AZ_MID = new Pose2d(
            2.645650625228882,
            0.4993302524089813,
            Rotation2d.kZero
    );

    public static final Pose2d RIGHT_TRENCHLINE_AZ_MID = mirror(LEFT_TRENCHLINE_AZ_MID);

    public static final Pose2d LEFT_SWIPE_ENDPOINT_FAR = new Pose2d(
            7.550732612609863,
            3.5814127922058105,
            Rotation2d.kCCW_90deg
    );

    public static final Pose2d LEFT_SWIPE_ENDPOINT_MID = new Pose2d(
            6.324461936950684,
            3.5814127922058105,
            Rotation2d.kCCW_90deg
    );

    public static final Pose2d RIGHT_SWIPE_ENDPOINT_FAR = mirror(LEFT_SWIPE_ENDPOINT_FAR);

    public static final Pose2d RIGHT_SWIPE_ENDPOINT_MID = mirror(LEFT_SWIPE_ENDPOINT_MID);

    public static final Pose2d LEFT_TRENCHLINE_TRENCH = new Pose2d(
            4.565030574798584,
            0.4993302524089813,
            Rotation2d.kZero
    );

    public static final Pose2d RIGHT_TRENCHLINE_TRENCH = mirror(LEFT_TRENCHLINE_TRENCH);

    private static Pose2d mirror(Pose2d leftSidePose) {
        return new Pose2d(
                leftSidePose.getX(),
                FieldConstants.fieldWidth - leftSidePose.getY(),
                leftSidePose.getRotation().times(-1)
        );
    }
}
