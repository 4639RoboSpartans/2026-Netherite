/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.shooter;

import org.team4639.lib.util.LoggedTunableNumber;

public class PIDs {
    public static final LoggedTunableNumber shooterKp = new LoggedTunableNumber("Shooter/kP").initDefault(0);
    public static final LoggedTunableNumber shooterKi = new LoggedTunableNumber("Shooter/kI").initDefault(0);
    public static final LoggedTunableNumber shooterKd = new LoggedTunableNumber("Shooter/kD").initDefault(0);
    public static final LoggedTunableNumber shooterKs = new LoggedTunableNumber("Shooter/kS").initDefault(0);
    public static final LoggedTunableNumber shooterKv = new LoggedTunableNumber("Shooter/kV").initDefault(0);
    public static final LoggedTunableNumber shooterKa = new LoggedTunableNumber("Shooter/kA").initDefault(0);

    public static final LoggedTunableNumber shooterKpSim = new LoggedTunableNumber("Shooter/kPSim").initDefault(0);
    public static final LoggedTunableNumber shooterKiSim = new LoggedTunableNumber("Shooter/kISim").initDefault(0);
    public static final LoggedTunableNumber shooterKdSim = new LoggedTunableNumber("Shooter/kDSim").initDefault(0);
    public static final LoggedTunableNumber shooterKsSim = new LoggedTunableNumber("Shooter/kSSim").initDefault(0);
    public static final LoggedTunableNumber shooterKvSim =
            new LoggedTunableNumber("Shooter/kVSim").initDefault(0.0017386);
    public static final LoggedTunableNumber shooterKaSim = new LoggedTunableNumber("Shooter/kASim").initDefault(0);
}
