/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.lib.util;

import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {
    private static List<VirtualSubsystem> instances = new ArrayList<>();

    public VirtualSubsystem() {
        instances.add(this);
    }

    /** This method is called periodically before the command scheduler. */
    public abstract void periodic();

    /**
     * This method is called periodically after the command scheduler, and should be used for applying
     * outputs.
     */
    public abstract void periodicAfterScheduler();

    /** Run the periodic methods for all subsystems. */
    public static void runAllPeriodic() {
        for (VirtualSubsystem instance : instances) {
            instance.periodic();
        }
    }

    /** Run the "after periodic" methods for all subsystems. */
    public static void runAllPeriodicAfterScheduler() {
        for (VirtualSubsystem instance : instances) {
            instance.periodicAfterScheduler();
        }
    }
}
