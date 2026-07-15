// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

final class FeedControlStateMachine {
    enum OutputMode {
        STOPPED,
        FULL,
        REDUCED
    }

    private enum State {
        WAITING_TO_START,
        FULL,
        REDUCED,
        PAUSED_FOR_FLYWHEEL
    }

    private final int initialReadyCycles;
    private final int reducedFeedCycles;
    private final int resumeReadyCycles;

    private State state = State.WAITING_TO_START;
    private int goodCycles;
    private int outOfRangeCycles;

    FeedControlStateMachine(
        int initialReadyCycles,
        int reducedFeedCycles,
        int resumeReadyCycles
    ) {
        this.initialReadyCycles = initialReadyCycles;
        this.reducedFeedCycles = reducedFeedCycles;
        this.resumeReadyCycles = resumeReadyCycles;
    }

    OutputMode update(boolean feedInterlocksReady, boolean flywheelReady) {
        if (!feedInterlocksReady) {
            reset();
            return OutputMode.STOPPED;
        }

        return switch (state) {
            case WAITING_TO_START -> updateWaitingToStart(flywheelReady);
            case FULL -> updateFull(flywheelReady);
            case REDUCED -> updateReduced(flywheelReady);
            case PAUSED_FOR_FLYWHEEL -> updatePausedForFlywheel(flywheelReady);
        };
    }

    void reset() {
        state = State.WAITING_TO_START;
        goodCycles = 0;
        outOfRangeCycles = 0;
    }

    int getGoodCycles() {
        return goodCycles;
    }

    int getOutOfRangeCycles() {
        return outOfRangeCycles;
    }

    private OutputMode updateWaitingToStart(boolean flywheelReady) {
        if (!flywheelReady) {
            goodCycles = 0;
            return OutputMode.STOPPED;
        }

        goodCycles++;
        if (goodCycles < initialReadyCycles) {
            return OutputMode.STOPPED;
        }

        state = State.FULL;
        return OutputMode.FULL;
    }

    private OutputMode updateFull(boolean flywheelReady) {
        if (flywheelReady) {
            return OutputMode.FULL;
        }

        state = State.REDUCED;
        outOfRangeCycles = 1;
        return OutputMode.REDUCED;
    }

    private OutputMode updateReduced(boolean flywheelReady) {
        if (flywheelReady) {
            state = State.FULL;
            outOfRangeCycles = 0;
            return OutputMode.FULL;
        }

        outOfRangeCycles++;
        if (outOfRangeCycles <= reducedFeedCycles) {
            return OutputMode.REDUCED;
        }

        state = State.PAUSED_FOR_FLYWHEEL;
        goodCycles = 0;
        return OutputMode.STOPPED;
    }

    private OutputMode updatePausedForFlywheel(boolean flywheelReady) {
        if (!flywheelReady) {
            goodCycles = 0;
            return OutputMode.STOPPED;
        }

        goodCycles++;
        if (goodCycles < resumeReadyCycles) {
            return OutputMode.STOPPED;
        }

        state = State.FULL;
        outOfRangeCycles = 0;
        return OutputMode.FULL;
    }
}
