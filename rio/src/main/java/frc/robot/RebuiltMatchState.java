package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Pure 2026 REBUILT shift timing and hub-state calculations. */
final class RebuiltMatchState {
    private RebuiltMatchState() {}

    static double shiftTimeRemainingSeconds(boolean teleop, double matchTimeSeconds) {
        if (!teleop || matchTimeSeconds < 0.0) {
            return 0.0;
        }

        if (matchTimeSeconds > 130.0) {
            return Math.min(10.0, matchTimeSeconds - 130.0);
        } else if (matchTimeSeconds > 105.0) {
            return matchTimeSeconds - 105.0;
        } else if (matchTimeSeconds > 80.0) {
            return matchTimeSeconds - 80.0;
        } else if (matchTimeSeconds > 55.0) {
            return matchTimeSeconds - 55.0;
        } else if (matchTimeSeconds > 30.0) {
            return matchTimeSeconds - 30.0;
        }

        return Math.max(0.0, matchTimeSeconds);
    }

    static boolean isHubActive(
        Optional<Alliance> alliance,
        boolean autonomousEnabled,
        boolean teleopEnabled,
        double matchTimeSeconds,
        String gameData
    ) {
        if (alliance.isEmpty()) {
            return false;
        }
        if (autonomousEnabled) {
            return true;
        }
        if (!teleopEnabled) {
            return false;
        }
        if (gameData.isEmpty()) {
            return true;
        }

        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return true;
            }
        }

        boolean shiftOneActive = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTimeSeconds > 130.0) {
            return true;
        } else if (matchTimeSeconds > 105.0) {
            return shiftOneActive;
        } else if (matchTimeSeconds > 80.0) {
            return !shiftOneActive;
        } else if (matchTimeSeconds > 55.0) {
            return shiftOneActive;
        } else if (matchTimeSeconds > 30.0) {
            return !shiftOneActive;
        }

        return true;
    }
}
