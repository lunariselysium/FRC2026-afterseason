// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

final class FeederUnjamController {
    private static final double kTimeComparisonEpsilonSeconds = 1.0e-9;

    private final double durationSeconds;

    private boolean active;
    private double startedAtSeconds;

    FeederUnjamController(double durationSeconds) {
        this.durationSeconds = durationSeconds;
    }

    boolean request(double timestampSeconds) {
        update(timestampSeconds);
        if (active) {
            return false;
        }

        active = true;
        startedAtSeconds = timestampSeconds;
        return true;
    }

    boolean update(double timestampSeconds) {
        if (active
            && timestampSeconds - startedAtSeconds
                >= durationSeconds - kTimeComparisonEpsilonSeconds) {
            active = false;
        }

        return active;
    }

    boolean isActive() {
        return active;
    }

    void cancel() {
        active = false;
        startedAtSeconds = 0.0;
    }
}
