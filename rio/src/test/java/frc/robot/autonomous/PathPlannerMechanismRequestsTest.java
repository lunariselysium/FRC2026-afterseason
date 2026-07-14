package frc.robot.autonomous;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class PathPlannerMechanismRequestsTest {
    @Test
    void recordsAnIntakeRollerStartRequest() {
        PathPlannerMechanismRequests requests = new PathPlannerMechanismRequests();

        requests.startIntakeRollers();

        assertTrue(requests.areIntakeRollersRequested());
    }

    @Test
    void clearsAllPersistentRequestsWhenAutonomousEnds() {
        PathPlannerMechanismRequests requests = new PathPlannerMechanismRequests();
        requests.startIntakeRollers();
        requests.startAutoScore();

        requests.clear();

        assertFalse(requests.areIntakeRollersRequested());
        assertFalse(requests.isAutoScoreRequested());
    }

    @Test
    void stoppingIntakeRollersDoesNotEndAutoScore() {
        PathPlannerMechanismRequests requests = new PathPlannerMechanismRequests();
        requests.startIntakeRollers();
        requests.startAutoScore();

        requests.stopIntakeRollers();

        assertFalse(requests.areIntakeRollersRequested());
        assertTrue(requests.isAutoScoreRequested());
    }

    @Test
    void endingAutoScoreDoesNotStopIntakeRollers() {
        PathPlannerMechanismRequests requests = new PathPlannerMechanismRequests();
        requests.startIntakeRollers();
        requests.startAutoScore();

        requests.endAutoScore();

        assertTrue(requests.areIntakeRollersRequested());
        assertFalse(requests.isAutoScoreRequested());
    }
}
