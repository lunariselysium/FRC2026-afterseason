package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

class RebuiltMatchStateTest {
    private static final double kTolerance = 1.0e-9;

    @Test
    void reportsTimeRemainingInCurrentTeleopShift() {
        assertEquals(10.0, RebuiltMatchState.shiftTimeRemainingSeconds(true, 140.0), kTolerance);
        assertEquals(5.0, RebuiltMatchState.shiftTimeRemainingSeconds(true, 135.0), kTolerance);
        assertEquals(25.0, RebuiltMatchState.shiftTimeRemainingSeconds(true, 130.0), kTolerance);
        assertEquals(12.0, RebuiltMatchState.shiftTimeRemainingSeconds(true, 117.0), kTolerance);
        assertEquals(30.0, RebuiltMatchState.shiftTimeRemainingSeconds(true, 30.0), kTolerance);
        assertEquals(0.0, RebuiltMatchState.shiftTimeRemainingSeconds(true, 0.0), kTolerance);
        assertEquals(0.0, RebuiltMatchState.shiftTimeRemainingSeconds(false, 100.0), kTolerance);
    }

    @Test
    void identifiesFiveSecondWarningAndShiftEndWindows() {
        assertTrue(RebuiltMatchState.isFiveSecondsBeforeShiftEnd(135.0));
        assertTrue(RebuiltMatchState.isFiveSecondsBeforeShiftEnd(134.9));
        assertFalse(RebuiltMatchState.isFiveSecondsBeforeShiftEnd(134.7));

        assertTrue(RebuiltMatchState.isImmediatelyAfterShiftEnd(130.0));
        assertTrue(RebuiltMatchState.isImmediatelyAfterShiftEnd(129.9));
        assertFalse(RebuiltMatchState.isImmediatelyAfterShiftEnd(129.7));
        assertTrue(RebuiltMatchState.isImmediatelyAfterShiftEnd(0.0));
    }

    @Test
    void reportsHubActiveFromAllianceGameDataAndShift() {
        assertFalse(RebuiltMatchState.isHubActive(
            Optional.empty(), false, true, 120.0, "R"
        ));
        assertTrue(RebuiltMatchState.isHubActive(
            Optional.of(Alliance.Red), true, false, 10.0, "R"
        ));
        assertFalse(RebuiltMatchState.isHubActive(
            Optional.of(Alliance.Red), false, false, 120.0, "R"
        ));
        assertTrue(RebuiltMatchState.isHubActive(
            Optional.of(Alliance.Red), false, true, 135.0, "R"
        ));
        assertTrue(RebuiltMatchState.isHubActive(
            Optional.of(Alliance.Red), false, true, 120.0, "B"
        ));
        assertFalse(RebuiltMatchState.isHubActive(
            Optional.of(Alliance.Red), false, true, 120.0, "R"
        ));
        assertTrue(RebuiltMatchState.isHubActive(
            Optional.of(Alliance.Red), false, true, 100.0, "R"
        ));
        assertFalse(RebuiltMatchState.isHubActive(
            Optional.of(Alliance.Red), false, true, 70.0, "R"
        ));
        assertTrue(RebuiltMatchState.isHubActive(
            Optional.of(Alliance.Red), false, true, 45.0, "R"
        ));
        assertTrue(RebuiltMatchState.isHubActive(
            Optional.of(Alliance.Red), false, true, 15.0, "R"
        ));
        assertTrue(RebuiltMatchState.isHubActive(
            Optional.of(Alliance.Blue), false, true, 120.0, "R"
        ));
    }

    @Test
    void assumesHubActiveWhenGameDataIsMissingOrInvalid() {
        assertTrue(RebuiltMatchState.isHubActive(
            Optional.of(Alliance.Red), false, true, 120.0, ""
        ));
        assertTrue(RebuiltMatchState.isHubActive(
            Optional.of(Alliance.Red), false, true, 120.0, "?"
        ));
    }
}
