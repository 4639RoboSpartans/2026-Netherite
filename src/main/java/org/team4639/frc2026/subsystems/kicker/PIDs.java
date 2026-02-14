package org.team4639.frc2026.subsystems.kicker;

import org.team4639.lib.util.LoggedTunableNumber;

public class PIDs {
    public static final LoggedTunableNumber kickerKP = new LoggedTunableNumber("Kicker/kP").initDefault(0.0);
    public static final LoggedTunableNumber kickerKI = new LoggedTunableNumber("Kicker/kI").initDefault(0.0);
    public static final LoggedTunableNumber kickerKD = new LoggedTunableNumber("Kicker/kD").initDefault(0.0);
    public static final LoggedTunableNumber kickerKS = new LoggedTunableNumber("Kicker/kS").initDefault(0.0);
    public static final LoggedTunableNumber kickerKV = new LoggedTunableNumber("Kicker/kV").initDefault(0.0);
    public static final LoggedTunableNumber kickerKA = new LoggedTunableNumber("Kicker/kA").initDefault(0.0);
}
