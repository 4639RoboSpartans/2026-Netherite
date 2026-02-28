/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.turret;

public class Constants {
    public static final double MOTOR_TO_TURRET_GEAR_RATIO =  12.0 / 28 * 18 / 40 * 12 / 92;
    public static final double SHARED_GEAR_TO_TURRET_GEAR_RATIO = 92.0 / 12;

    // As viewed from above, intake facing up
    public static final double SHARED_GEAR_TEETH = 40;
    public static final double LEFT_ENCODER_GEAR_TEETH = 41;
    public static final double RIGHT_ENCODER_GEAR_TEETH = 40;

    public static final double LEFT_ENCODER_OFFSET = -0.619385;
    public static final double RIGHT_ENCODER_OFFSET = -0.939697;

    public static final boolean LEFT_ENCODER_INVERTED = true;
    public static final boolean RIGHT_ENCODER_INVERTED = true;

    public static final double TURRET_MIN_ROTATIONS = 0;
    public static final double TURRET_MAX_ROTATIONS = 0.875;

    public static final double TURRET_EXTENDED_MIN_ROTATIONS = 0;
    public static final double TURRET_EXTENDED_MAX_ROTATIONS = 0.75;

    public static final double ROTOR_ROTATION_TOLERANCE = 100;
}
