/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.led;

import edu.wpi.first.wpilibj.util.Color;

public class LEDHelper {
    public static int[] toRGBArray(Color color) {
        return new int[] {(int) color.red, (int) color.green, (int) color.blue};
    }
}
