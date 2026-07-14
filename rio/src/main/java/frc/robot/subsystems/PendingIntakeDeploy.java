package frc.robot.subsystems;

/** Remembers a deploy request made before intake position control is available. */
final class PendingIntakeDeploy {
    private boolean requested;

    void request() {
        requested = true;
    }

    boolean consumeWhenAllowed(boolean positionControlAllowed) {
        if (!requested || !positionControlAllowed) {
            return false;
        }

        requested = false;
        return true;
    }

    void clear() {
        requested = false;
    }
}
