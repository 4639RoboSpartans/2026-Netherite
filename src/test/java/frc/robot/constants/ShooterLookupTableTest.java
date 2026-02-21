/* Copyright (c) 2025-2026 FRC 4639. */

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team4639.frc2026.constants.shooter.ScoringState;
import org.team4639.frc2026.constants.shooter.ShooterLookupTable;

public class ShooterLookupTableTest {

    ShooterLookupTable shooterLookupTable;
    final static InterpolatingDoubleTreeMap distToRPM = new  InterpolatingDoubleTreeMap();
    final static InterpolatingDoubleTreeMap distToAngle = new  InterpolatingDoubleTreeMap();
    final static InterpolatingDoubleTreeMap distToTOF = new  InterpolatingDoubleTreeMap();
    final static Pose2d robotPose = new Pose2d(
            new Translation2d(
                    0.1, 0.5
            ),
            new Rotation2d()
    );
    final static Translation2d hubTranslation =  new Translation2d(4.623, 4.028);
    final static Translation2d chassisSpeeds = new Translation2d(1, -3);

    static {
        LoadShooterLookupTableData.loadData(distToRPM, distToAngle, distToTOF);
    }

    @BeforeEach
    public void setup() {
        shooterLookupTable = new ShooterLookupTable(
            distToRPM,
            distToAngle,
            distToTOF
        );
    }

    @Test
    public void shooterLookupTableTest() {
        ScoringState convergeState1 = shooterLookupTable.convergeShooterStateSOTFTurret(robotPose, hubTranslation, chassisSpeeds, 5);
        System.out.println("shooter state: " + convergeState1);
        ScoringState convergeState2 = shooterLookupTable.convergeShooterStateSOTFTurret(robotPose, hubTranslation, chassisSpeeds, 10);
        System.out.println("shooter state: " + convergeState2);
    }
}
