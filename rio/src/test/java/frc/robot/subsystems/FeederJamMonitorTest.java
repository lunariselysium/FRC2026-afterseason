// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class FeederJamMonitorTest {
    @Test
    void reportsAWarningOnlyAfterOneMotorStaysJammedForQualificationTime() {
        FeederJamMonitor monitor = new FeederJamMonitor(0.20);

        assertFalse(monitor.update(1.00, true, true, false, false, false));
        assertFalse(monitor.update(1.19, true, true, false, false, false));
        assertTrue(monitor.update(1.20, true, true, false, false, false));
        assertTrue(monitor.update(1.30, true, true, false, false, false));
    }

    @Test
    void stopsWarningWhenTheQualifiedJamClears() {
        FeederJamMonitor monitor = new FeederJamMonitor(0.20);

        monitor.update(1.00, true, true, false, false, false);
        assertTrue(monitor.update(1.20, true, true, false, false, false));
        assertFalse(monitor.update(1.21, true, false, false, false, false));
    }

    @Test
    void requiresOneMotorToRemainContinuouslyJammed() {
        FeederJamMonitor monitor = new FeederJamMonitor(0.20);

        monitor.update(1.00, true, true, false, false, false);
        monitor.update(1.15, true, false, false, false, false);

        assertFalse(monitor.update(1.25, true, true, false, false, false));
        assertFalse(monitor.update(1.44, true, true, false, false, false));
        assertTrue(monitor.update(1.45, true, true, false, false, false));
    }

    @Test
    void anyOfTheFourFeederMotorsCanTriggerTheWarning() {
        for (int jammedMotorIndex = 0; jammedMotorIndex < 4; jammedMotorIndex++) {
            FeederJamMonitor monitor = new FeederJamMonitor(0.20);

            updateWithOnlyOneMotorJammed(monitor, 1.00, jammedMotorIndex);

            assertTrue(
                updateWithOnlyOneMotorJammed(monitor, 1.20, jammedMotorIndex),
                "Motor index " + jammedMotorIndex + " did not trigger the warning"
            );
        }
    }

    @Test
    void doesNotMonitorWhenNoForwardFeedIsRequested() {
        FeederJamMonitor monitor = new FeederJamMonitor(0.20);

        monitor.update(1.00, true, true, false, false, false);

        assertFalse(monitor.update(1.20, false, true, false, false, false));
    }

    private boolean updateWithOnlyOneMotorJammed(
        FeederJamMonitor monitor,
        double timestampSeconds,
        int jammedMotorIndex
    ) {
        return monitor.update(
            timestampSeconds,
            true,
            jammedMotorIndex == 0,
            jammedMotorIndex == 1,
            jammedMotorIndex == 2,
            jammedMotorIndex == 3
        );
    }
}
