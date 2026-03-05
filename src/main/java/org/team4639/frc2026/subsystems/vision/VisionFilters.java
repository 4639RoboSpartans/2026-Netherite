/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.vision;

import static edu.wpi.first.units.Units.Rotations;
import static org.team4639.frc2026.subsystems.vision.VisionConstants.*;

import java.util.function.Predicate;
import org.team4639.frc2026.subsystems.vision.VisionIO.PoseObservation;
import lombok.Getter;

@Getter
public enum VisionFilters {
    AMBIGUITY(observation -> observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity),
    TAG_COUNT(observation -> observation.tagCount() < 1),
    MAX_Z_ERROR(observation -> Math.abs(observation.pose().getZ()) > maxZError),
    FIELD_BOUNDARIES(observation -> observation.pose().getX() < 0.0
            || observation.pose().getX() > aprilTagLayout.getFieldLength()
            || observation.pose().getY() < 0.0
            || observation.pose().getY() > aprilTagLayout.getFieldWidth()),
    ROTS3D(observation -> {
        return observation.pose().getRotation().getMeasureX().abs(Rotations) > 0.02 ||
        observation.pose().getRotation().getMeasureY().abs(Rotations) > 0.02;
    });
    /*DISTANCE(observation -> {
        return observation.averageTagDistance() > 3;
    });*/

    /**
     * Returns true if we want to reject the pose and false if we keep it
     */
    private final Predicate<PoseObservation> test;

    VisionFilters(Predicate<PoseObservation> test) {
        this.test = test;
    }
}
