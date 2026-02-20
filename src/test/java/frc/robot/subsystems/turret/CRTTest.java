/* Copyright (c) 2025-2026 FRC 4639. */

package frc.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.subsystems.turret.EncoderIOSim;
import org.team4639.frc2026.subsystems.turret.Turret;
import org.team4639.frc2026.subsystems.turret.TurretIOSim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

public class CRTTest {
    Turret turret;
    private Angle readingTolerance = Degrees.of(0.001);
    private double precision = 10.0;
    private Angle absoluteEncoder1Reading = Degrees.of(0);
    private Angle absoluteEncoder2Reading = Degrees.of(0);

    private Angle getAbs1() {
        return Degrees.of(absoluteEncoder1Reading.in(Degrees) % 360.0);
    }

    private Angle getAbs2() {
        return Degrees.of(absoluteEncoder2Reading.in(Degrees) % 360.0);
    }

    @BeforeEach
    public void setup() {
        turret = new Turret(
                new TurretIOSim(),
                new EncoderIOSim(),
                new EncoderIOSim(),
                RobotState.getInstance()
        );
    }

    @Test
    public void testCRT() {
        System.out.println("Starting CRT Test");
        double commonRatio = 92.0 / 12; // 60 -> 180
        int driveGearTeeth = 40; // shared gear driving both encoders (60 in this case)
        int encoder1Pinion = 41; // 19t attached to 60t drive gear
        int encoder2Pinion = 40; // 21t attached to 60t drive gear
        // Limit the sweep to the unique coverage
        double coverageRotations = 3.0;
        double sweepRotations = coverageRotations - 0.05;
        int maxIterations = (int) Math.round(sweepRotations * 360 * precision);
        for (int i = 0; i < maxIterations; i++) {
            var turretAngle = Degrees.of(i / precision);
            absoluteEncoder1Reading = turretAngle.times(commonRatio * driveGearTeeth / encoder1Pinion);
            absoluteEncoder2Reading = turretAngle.times(commonRatio * driveGearTeeth / encoder2Pinion);
            var estimatedAngle = Rotations.of(turret.getTurretRotation(absoluteEncoder1Reading.in(Rotations) % 1, absoluteEncoder2Reading.in(Rotations) % 1));
            var testing = turretAngle.isNear(estimatedAngle, readingTolerance);
            if (!testing) {
                System.out.println("Absolute Encoder Reading: " + getAbs1() + " " + getAbs2());
                System.out.println("Turret Angle(degrees): " + turretAngle.in(Degrees));
                System.out.println("CRT Angle(degrees): " + estimatedAngle.in(Degrees));
                break;
            }
            Assertions.assertTrue(testing);
        }
    }
}
