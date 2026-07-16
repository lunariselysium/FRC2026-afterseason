package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** One-shot PathPlanner commands for intake rollers that may run alongside intake motion. */
public final class PathPlannerIntakeRollerCommands {
    private PathPlannerIntakeRollerCommands() {}

    public static Command startCommand(Runnable startRollers) {
        return Commands.runOnce(startRollers);
    }
}
