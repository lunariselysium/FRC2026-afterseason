package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class AutonomousIntakeRollerPolicyTest {
    @Test
    void runsRollersOnlyWhileAutonomousIsEnabled() {
        assertTrue(AutonomousIntakeRollerPolicy.shouldRunRollers(true));
        assertFalse(AutonomousIntakeRollerPolicy.shouldRunRollers(false));
    }
}
