package frc.robot.autonomous;

/** Tracks mechanism modes requested by PathPlanner named commands. */
public class PathPlannerMechanismRequests {
    private boolean intakeRollersRequested;
    private boolean autoScoreRequested;

    public void startIntakeRollers() {
        intakeRollersRequested = true;
    }

    public void stopIntakeRollers() {
        intakeRollersRequested = false;
    }

    public boolean areIntakeRollersRequested() {
        return intakeRollersRequested;
    }

    public void startAutoScore() {
        autoScoreRequested = true;
    }

    public void endAutoScore() {
        autoScoreRequested = false;
    }

    public boolean isAutoScoreRequested() {
        return autoScoreRequested;
    }

    public void clear() {
        intakeRollersRequested = false;
        autoScoreRequested = false;
    }
}
