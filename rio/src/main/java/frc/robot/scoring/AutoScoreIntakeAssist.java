// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.scoring;

public class AutoScoreIntakeAssist {
    public enum IntakeRequest {
        IDLE,
        DEPLOYED,
        AUTO_SCORE_RETRACTED
    }

    private boolean autoScoreWasActive;
    private boolean feedingStarted;

    public IntakeRequest update(
        boolean autoScoreActive,
        boolean autoScoreFeeding,
        boolean intakeButtonPressed
    ) {
        if (!autoScoreActive) {
            boolean shouldRestoreDeployed = autoScoreWasActive && feedingStarted;
            autoScoreWasActive = false;
            feedingStarted = false;
            return shouldRestoreDeployed ? IntakeRequest.DEPLOYED : IntakeRequest.IDLE;
        }

        autoScoreWasActive = true;
        feedingStarted = feedingStarted || autoScoreFeeding;
        if (!feedingStarted) {
            return IntakeRequest.IDLE;
        }

        return intakeButtonPressed
            ? IntakeRequest.DEPLOYED
            : IntakeRequest.AUTO_SCORE_RETRACTED;
    }
}
