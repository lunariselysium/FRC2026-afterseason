// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.scoring;

import frc.robot.Constants.IntakeConstants;

public class AutoScoreIntakeAssist {
    public enum IntakeRequest {
        IDLE(false),
        STOWED(true),
        DEPLOYED(true),
        AUTO_SCORE_SEMI_DEPLOYED(true),
        AUTO_SCORE_SEVENTY_PERCENT_DEPLOYED(true);

        private final boolean runsRollers;

        IntakeRequest(boolean runsRollers) {
            this.runsRollers = runsRollers;
        }

        public boolean runsRollers() {
            return runsRollers;
        }
    }

    private enum ManualPositionPriority {
        NONE,
        DEPLOY,
        RETRACT
    }

    private boolean autoScoreWasActive;
    private boolean feedingStarted;
    private double feedingStartedAtSeconds;
    private boolean deployButtonWasPressed;
    private boolean retractButtonWasPressed;
    private ManualPositionPriority manualPositionPriority = ManualPositionPriority.NONE;

    public IntakeRequest update(
        boolean autoScoreActive,
        boolean autoScoreFeeding,
        boolean deployButtonPressed,
        double timestampSeconds
    ) {
        return update(
            autoScoreActive,
            autoScoreFeeding,
            deployButtonPressed,
            false,
            timestampSeconds
        );
    }

    public IntakeRequest update(
        boolean autoScoreActive,
        boolean autoScoreFeeding,
        boolean deployButtonPressed,
        boolean retractButtonPressed,
        double timestampSeconds
    ) {
        IntakeRequest autoScoreRequest = updateAutoScoreRequest(
            autoScoreActive,
            autoScoreFeeding,
            timestampSeconds
        );
        updateManualPositionPriority(deployButtonPressed, retractButtonPressed);

        return switch (manualPositionPriority) {
            case DEPLOY -> IntakeRequest.DEPLOYED;
            case RETRACT -> IntakeRequest.STOWED;
            case NONE -> autoScoreRequest;
        };
    }

    public boolean hasFeedingStarted() {
        return feedingStarted;
    }

    private IntakeRequest updateAutoScoreRequest(
        boolean autoScoreActive,
        boolean autoScoreFeeding,
        double timestampSeconds
    ) {
        if (!autoScoreActive) {
            boolean shouldRestoreDeployed = autoScoreWasActive && feedingStarted;
            autoScoreWasActive = false;
            feedingStarted = false;
            feedingStartedAtSeconds = 0.0;
            return shouldRestoreDeployed ? IntakeRequest.DEPLOYED : IntakeRequest.IDLE;
        }

        autoScoreWasActive = true;
        if (autoScoreFeeding && !feedingStarted) {
            feedingStarted = true;
            feedingStartedAtSeconds = timestampSeconds;
        }
        if (!feedingStarted) {
            return IntakeRequest.IDLE;
        }

        double elapsedSinceFeedingStartedSeconds = timestampSeconds - feedingStartedAtSeconds;
        if (elapsedSinceFeedingStartedSeconds < IntakeConstants.kAutoScoreRetractionDelaySeconds) {
            return IntakeRequest.IDLE;
        }

        double oscillationSeconds =
            elapsedSinceFeedingStartedSeconds - IntakeConstants.kAutoScoreRetractionDelaySeconds;
        long oscillationPhase = (long) Math.floor(
            oscillationSeconds / IntakeConstants.kAutoScoreOscillationPhaseSeconds
        );
        return oscillationPhase % 2L == 0L
            ? IntakeRequest.AUTO_SCORE_SEMI_DEPLOYED
            : IntakeRequest.AUTO_SCORE_SEVENTY_PERCENT_DEPLOYED;
    }

    private void updateManualPositionPriority(
        boolean deployButtonPressed,
        boolean retractButtonPressed
    ) {
        boolean deployButtonPressedNow = deployButtonPressed && !deployButtonWasPressed;
        boolean retractButtonPressedNow = retractButtonPressed && !retractButtonWasPressed;

        if (deployButtonPressedNow && !retractButtonPressedNow) {
            manualPositionPriority = ManualPositionPriority.DEPLOY;
        } else if (retractButtonPressedNow) {
            // Simultaneous presses cannot be ordered within one robot loop; retract is safer.
            manualPositionPriority = ManualPositionPriority.RETRACT;
        }

        if (manualPositionPriority == ManualPositionPriority.DEPLOY && !deployButtonPressed) {
            manualPositionPriority = retractButtonPressed
                ? ManualPositionPriority.RETRACT
                : ManualPositionPriority.NONE;
        } else if (
            manualPositionPriority == ManualPositionPriority.RETRACT
                && !retractButtonPressed
        ) {
            manualPositionPriority = deployButtonPressed
                ? ManualPositionPriority.DEPLOY
                : ManualPositionPriority.NONE;
        }

        deployButtonWasPressed = deployButtonPressed;
        retractButtonWasPressed = retractButtonPressed;
    }
}
