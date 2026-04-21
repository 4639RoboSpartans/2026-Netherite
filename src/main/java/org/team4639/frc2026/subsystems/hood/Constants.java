/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

public class Constants {
    public static final double ENCODER_TO_PIVOT_GEAR_RATIO = 30.0 / 324;
    public static final double MOTOR_TO_ENCODER_GEAR_RATIO = 12.0 / 56 * 15 / 30;
    public static final double MOTOR_TO_HOOD_GEAR_RATIO = MOTOR_TO_ENCODER_GEAR_RATIO * ENCODER_TO_PIVOT_GEAR_RATIO;
    public static final double ENCODER_ROTATIONS_PER_DEGREE = ENCODER_TO_PIVOT_GEAR_RATIO / 360.0;
    public static final double HOOD_MIN_ANGLE_DEGREES = 20;

    public static final double HOOD_ENCODER_MIN_ROTATION = -0.;
    public static final double HOOD_ENCODER_MAX_ROTATION = 0.;
    public static final double HOOD_RANGE_DEGREES = 30;
}
