/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.zone;

import org.team4639.frc2026.FieldConstants;
import org.team4639.lib.zone.Zones;

public class RebuiltZones {
    public static final Zones.Zone OUR_ALLIANCE_ZONE = new Zones.BaseZone(0, FieldConstants.LinesVertical.allianceZone, 0, FieldConstants.fieldWidth);
    public static final Zones.Zone NEUTRAL_ZONE = new Zones.BaseZone(FieldConstants.LinesVertical.allianceZone, FieldConstants.LinesVertical.oppAllianceZone, 0, FieldConstants.fieldWidth);
    public static final Zones.Zone NOT_OUR_ALLIANCE_ZONE = new Zones.BaseZone(FieldConstants.LinesVertical.allianceZone, FieldConstants.fieldLength, 0, FieldConstants.fieldWidth);
}
