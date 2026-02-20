/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.ports;

import org.team4639.frc2026.util.CanDeviceId;
import org.team4639.frc2026.util.PortConfiguration;

public class Netherite {
    public static final PortConfiguration portConfiguration = new PortConfiguration();

    static {
        portConfiguration.IntakeExtensionMotorID = new CanDeviceId(20);
        portConfiguration.IntakeRollersMotorID = new CanDeviceId(21);
        portConfiguration.SpindexerMotorID = new  CanDeviceId(22);
        portConfiguration.KickerMotorID = new CanDeviceId(23);
        portConfiguration.TurretMotorID = new CanDeviceId(24);
        portConfiguration.TurretLeftEncoderID = new CanDeviceId(25);
        portConfiguration.TurretRightEncoderID = new CanDeviceId(26);
        portConfiguration.HoodMotorID = new CanDeviceId(27);
        portConfiguration.HoodEncoderID = new CanDeviceId(28);
        portConfiguration.shooterMotorLeftID = new CanDeviceId(29);
        portConfiguration.shooterMotorRightID = new CanDeviceId(30);
    }
}
