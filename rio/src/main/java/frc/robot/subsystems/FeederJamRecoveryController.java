// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

final class FeederJamRecoveryController {
    enum OutputMode {
        NOMINAL,
        REVERSE
    }

    private static final double kTimeComparisonEpsilonSeconds = 1.0e-9;
    private static final int kMonitoredMotorCount = 4;

    private final double qualificationSeconds;
    private final double reverseSeconds;
    private final double[] jamStartedAtSeconds = new double[kMonitoredMotorCount];

    private boolean recoveryActive;
    private double recoveryStartedAtSeconds;

    FeederJamRecoveryController(double qualificationSeconds, double reverseSeconds) {
        this.qualificationSeconds = qualificationSeconds;
        this.reverseSeconds = reverseSeconds;
        clearJamTimers();
    }

    OutputMode update(
        double timestampSeconds,
        boolean recoveryAllowed,
        boolean floorJammed,
        boolean handoffWheelJammed,
        boolean beltLeaderJammed,
        boolean beltFollowerJammed
    ) {
        if (!recoveryAllowed) {
            reset();
            return OutputMode.NOMINAL;
        }

        if (recoveryActive) {
            if (hasElapsed(timestampSeconds, recoveryStartedAtSeconds, reverseSeconds)) {
                recoveryActive = false;
                clearJamTimers();
                return OutputMode.NOMINAL;
            }

            return OutputMode.REVERSE;
        }

        boolean[] jammed = {
            floorJammed,
            handoffWheelJammed,
            beltLeaderJammed,
            beltFollowerJammed
        };
        for (int motorIndex = 0; motorIndex < jammed.length; motorIndex++) {
            if (!jammed[motorIndex]) {
                jamStartedAtSeconds[motorIndex] = Double.NaN;
                continue;
            }

            if (Double.isNaN(jamStartedAtSeconds[motorIndex])) {
                jamStartedAtSeconds[motorIndex] = timestampSeconds;
                continue;
            }

            if (hasElapsed(
                timestampSeconds,
                jamStartedAtSeconds[motorIndex],
                qualificationSeconds
            )) {
                recoveryActive = true;
                recoveryStartedAtSeconds = timestampSeconds;
                clearJamTimers();
                return OutputMode.REVERSE;
            }
        }

        return OutputMode.NOMINAL;
    }

    boolean isRecoveryActive() {
        return recoveryActive;
    }

    void reset() {
        recoveryActive = false;
        recoveryStartedAtSeconds = 0.0;
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
