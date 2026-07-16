package frc.robot.autonomous;

/** Determines when intake rollers should run without an explicit PathPlanner marker. */
public final class AutonomousIntakeRollerPolicy {
    private AutonomousIntakeRollerPolicy() {}

    public static boolean shouldRunRollers(boolean autonomousEnabled) {
        return autonomousEnabled;
    }
}
