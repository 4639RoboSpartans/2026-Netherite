/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import java.util.function.Supplier;

public class VisionIOLimelight4 extends VisionIOLimelight {
    public String name;

    public VisionIOLimelight4(String name, Supplier<Rotation2d> rotationSupplier) {
        super(name, rotationSupplier);
        this.name = name;

        NetworkTableInstance.getDefault().getTable(name).getEntry("imumode_set").setInteger(0);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        super.updateInputs(inputs);
        if (RobotState.isDisabled()) {
            // Throttle Limelights so temperature dont rise
            NetworkTableInstance.getDefault()
                    .getTable(name)
                    .getEntry("throttle_set")
                    .setInteger(0);
        } else {
            NetworkTableInstance.getDefault()
                    .getTable(name)
                    .getEntry("throttle_set")
                    .setInteger(0);
        }
    }
}
