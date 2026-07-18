package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Publishes the current 2026 REBUILT shift and hub state for the drive team. */
final class RebuiltMatchStatePublisher {
    void update() {
        double matchTimeSeconds = DriverStation.getMatchTime();

        SmartDashboard.putNumber(
            "Match/ShiftTimeRemainingSeconds",
            RebuiltMatchState.shiftTimeRemainingSeconds(
                DriverStation.isTeleopEnabled(),
                matchTimeSeconds
            )
        );
        SmartDashboard.putBoolean(
            "Match/HubActive",
            RebuiltMatchState.isHubActive(
                DriverStation.getAlliance(),
                DriverStation.isAutonomousEnabled(),
                DriverStation.isTeleopEnabled(),
                matchTimeSeconds,
                DriverStation.getGameSpecificMessage()
            )
        );
    }
}
