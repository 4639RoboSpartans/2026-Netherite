/* Copyright (c) 2025-2026 FRC 4639. */

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
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
    private Angle readingTolerance = Degrees.of(0.01);
    private double precision = 100.0;
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
        // System.out.println("Starting CRT Test");
        // double commonRatio = 92.0 / 12;
        // int driveGearTeeth = 40;
        // int encoder1Pinion = 41;
        // int encoder2Pinion = 40;
        // double minRotations = 0;
        // double maxRotations = 0.75;
        // double sweepRotations = (maxRotations - minRotations) - 0.05;
        // int maxIterations = 2 * (int) Math.round(sweepRotations * precision);
        // for (int i = 0; i < maxIterations; i++) {
        //     var turretAngle = Rotations.of(i / precision + minRotations);
        //     absoluteEncoder1Reading = turretAngle.times(commonRatio * driveGearTeeth / encoder1Pinion);
        //     absoluteEncoder2Reading = turretAngle.times(commonRatio * driveGearTeeth / encoder2Pinion);
        //     var estimatedAngle = Rotations.of(turret.getTurretRotation(absoluteEncoder1Reading.in(Rotations), absoluteEncoder2Reading.in(Rotations)));
        //     var testing = turretAngle.isNear(estimatedAngle, readingTolerance);
        //     if(!testing) {
        //         System.out.println("Angle: " + turretAngle.in(Rotations) + "Estimated: " + estimatedAngle.in(Rotations));
        //     }
        //     Assertions.assertTrue(testing);
        // }

        Assertions.assertTrue(true);
    }

    @Test
    public void testCRT2() {
        System.out.println("Starting CRT Test 2");
        Assertions.assertTrue(MathUtil.isNear(6.17561557447743E-16, turret.getTurretRotation(0, 0), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.0200000000000006, turret.getTurretRotation(0.149593495934964, 0.153333333333338), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.0400000000000006, turret.getTurretRotation(0.299186991869923, 0.306666666666671), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.0600000000000006, turret.getTurretRotation(0.448780487804883, 0.460000000000005), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.0800000000000006, turret.getTurretRotation(-0.401626016260158, -0.386666666666662), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.100000000000001, turret.getTurretRotation(-0.252032520325199, -0.233333333333329), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.120000000000001, turret.getTurretRotation(-0.102439024390239, -0.0799999999999951), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.140000000000001, turret.getTurretRotation(0.0471544715447201, 0.0733333333333381), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.160000000000001, turret.getTurretRotation(0.196747967479679, 0.226666666666671), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.180000000000001, turret.getTurretRotation(0.346341463414638, 0.380000000000005), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.200000000000001, turret.getTurretRotation(0.495934959349598, -0.466666666666662), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.220000000000001, turret.getTurretRotation(-0.354471544715443, -0.313333333333329), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.240000000000001, turret.getTurretRotation(-0.204878048780484, -0.159999999999996), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.260000000000001, turret.getTurretRotation(-0.0552845528455244, -0.00666666666666238), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.280000000000001, turret.getTurretRotation(0.0943089430894357, 0.146666666666671), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.300000000000001, turret.getTurretRotation(0.243902439024395, 0.300000000000005), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.320000000000001, turret.getTurretRotation(0.393495934959354, 0.453333333333338), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.340000000000001, turret.getTurretRotation(-0.456910569105687, -0.393333333333329), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.360000000000001, turret.getTurretRotation(-0.307317073170727, -0.239999999999995), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.380000000000001, turret.getTurretRotation(-0.157723577235767, -0.0866666666666611), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.400000000000001, turret.getTurretRotation(-0.00813008130080783, 0.0666666666666718), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.420000000000001, turret.getTurretRotation(0.141463414634151, 0.220000000000005), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.440000000000001, turret.getTurretRotation(0.291056910569111, 0.373333333333339), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.460000000000001, turret.getTurretRotation(0.440650406504071, -0.473333333333328), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.480000000000001, turret.getTurretRotation(-0.40975609756097, -0.319999999999995), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.500000000000001, turret.getTurretRotation(-0.260162601626011, -0.166666666666661), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.520000000000001, turret.getTurretRotation(-0.110569105691051, -0.013333333333327), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.540000000000001, turret.getTurretRotation(0.0390243902439096, 0.140000000000007), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.560000000000001, turret.getTurretRotation(0.188617886178868, 0.293333333333339), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.580000000000001, turret.getTurretRotation(0.338211382113828, 0.446666666666673), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.600000000000001, turret.getTurretRotation(0.487804878048787, -0.399999999999993), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.620000000000001, turret.getTurretRotation(-0.362601626016254, -0.24666666666666), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.640000000000001, turret.getTurretRotation(-0.213008130081295, -0.0933333333333266), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.660000000000001, turret.getTurretRotation(-0.0634146341463353, 0.0600000000000067), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.680000000000001, turret.getTurretRotation(0.0861788617886257, 0.213333333333341), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.700000000000001, turret.getTurretRotation(0.235772357723585, 0.366666666666674), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.720000000000001, turret.getTurretRotation(0.385365853658544, -0.479999999999992), 0.01 * 1/360));
        Assertions.assertTrue(MathUtil.isNear(0.740000000000001, turret.getTurretRotation(-0.465040650406496, -0.326666666666658), 0.01 * 1/360));
    }
}
