// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

final class FeederJamMonitor {
    private static final double kTimeComparisonEpsilonSeconds = 1.0e-9;
    private static final int kMonitoredMotorCount = 4;

    private final double qualificationSeconds;
    private final double[] jamStartedAtSeconds = new double[kMonitoredMotorCount];

    private boolean jamWarningActive;

    FeederJamMonitor(double qualificationSeconds) {
        this.qualificationSeconds = qualificationSeconds;
        clearJamTimers();
    }

    boolean update(
        double timestampSeconds,
        boolean monitoringAllowed,
        boolean floorJammed,
        boolean handoffWheelJammed,
        boolean beltLeaderJammed,
        boolean beltFollowerJammed
    ) {
        if (!monitoringAllowed) {
            reset();
            return false;
        }

        boolean[] jammed = {
            floorJammed,
            handoffWheelJammed,
            beltLeaderJammed,
            beltFollowerJammed
        };
        boolean anyJamConditionActive = false;
        for (int motorIndex = 0; motorIndex < jammed.length; motorIndex++) {
            if (!jammed[motorIndex]) {
                jamStartedAtSeconds[motorIndex] = Double.NaN;
                continue;
            }

            anyJamConditionActive = true;
            if (Double.isNaN(jamStartedAtSeconds[motorIndex])) {
                jamStartedAtSeconds[motorIndex] = timestampSeconds;
                continue;
            }

            if (hasElapsed(
                timestampSeconds,
                jamStartedAtSeconds[motorIndex],
                qualificationSeconds
            )) {
                jamWarningActive = true;
            }
        }

        if (!anyJamConditionActive) {
            jamWarningActive = false;
        }

        return jamWarningActive;
    }

    boolean isJamWarningActive() {
        return jamWarningActive;
    }

    void reset() {
        jamWarningActive = false;
        clearJamTimers();
    }

    private boolean hasElapsed(
        double timestampSeconds,
        double startedAtSeconds,
        double durationSeconds
    ) {
        return timestampSeconds - startedAtSeconds
            >= durationSeconds - kTimeComparisonEpsilonSeconds;
    }

    private void clearJamTimers() {
        Arrays.fill(jamStartedAtSeconds, Double.NaN);
    }
}
