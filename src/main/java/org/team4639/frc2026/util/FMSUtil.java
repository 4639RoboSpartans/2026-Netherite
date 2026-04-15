/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.util;

import edu.wpi.first.wpilibj.DriverStation;

public class FMSUtil {
    public enum GameDataStatus {
        OK,
        NO_DS_ALLIANCE,
        NO_GAME_DATA
    }

    public record FMSGameData(DriverStation.Alliance wonAuto, GameDataStatus status) {}

    public static FMSGameData getAutoWinningAlliance() {
        String message = DriverStation.getGameSpecificMessage();
        DriverStation.Alliance wonAuto = null;
        GameDataStatus status;
        if (message.length() > 0) {
            char character = message.charAt(0);
            if (character == 'R') {
                wonAuto = DriverStation.Alliance.Blue;
            } else if (character == 'B') {
                wonAuto = DriverStation.Alliance.Red;
            }
        }

        if (wonAuto == null) status = GameDataStatus.NO_GAME_DATA;
        else if (!DriverStation.getAlliance().isPresent()) status = GameDataStatus.NO_DS_ALLIANCE;
        else status = GameDataStatus.OK;

        return new FMSGameData(wonAuto, status);
    }
}
