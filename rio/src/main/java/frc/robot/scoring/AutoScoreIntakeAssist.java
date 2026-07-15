// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.scoring;

import frc.robot.Constants.IntakeConstants;

public class AutoScoreIntakeAssist {
    public enum IntakeRequest {
        IDLE(false),
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

    private boolean autoScoreWasActive;
    private boolean feedingStarted;
    private double feedingStartedAtSeconds;

    public IntakeRequest update(
        boolean autoScoreActive,
        boolean autoScoreFeeding,
        boolean intakeButtonPressed,
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

        if (intakeButtonPressed) {
            return IntakeRequest.DEPLOYED;
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
}
