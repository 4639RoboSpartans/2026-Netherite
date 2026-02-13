/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.util;

import com.ctre.phoenix6.CANBus;
import lombok.Getter;

public class CanDeviceId {
    @Getter
    private final int deviceNumber;

    private final CANBus bus;

    public CanDeviceId(int deviceNumber, CANBus bus) {
        this.deviceNumber = deviceNumber;
        this.bus = bus;
    }

    // Use the default bus name "rio".
    public CanDeviceId(int deviceNumber) {
        this(deviceNumber, CANBus.roboRIO());
    }

    public CANBus getBus() {
        return bus;
    }

    @SuppressWarnings("NonOverridingEquals")
    public boolean equals(CanDeviceId other) {
        return other.deviceNumber == deviceNumber && other.bus.equals(bus);
    }

    @Override
    public String toString() {
        return "CanDeviceId(" + deviceNumber + ", " + bus.getName() + ")";
    }
}
