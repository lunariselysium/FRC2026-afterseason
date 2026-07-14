package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class PendingIntakeDeployTest {
    @Test
    void holdsDeployRequestUntilPositionControlIsAllowed() {
        PendingIntakeDeploy pendingDeploy = new PendingIntakeDeploy();
        pendingDeploy.request();

        assertFalse(pendingDeploy.consumeWhenAllowed(false));
        assertTrue(pendingDeploy.consumeWhenAllowed(true));
        assertFalse(pendingDeploy.consumeWhenAllowed(true));
    }
}
