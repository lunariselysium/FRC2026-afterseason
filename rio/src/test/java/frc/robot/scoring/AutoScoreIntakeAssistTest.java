// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.scoring;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.scoring.AutoScoreIntakeAssist.IntakeRequest;

class AutoScoreIntakeAssistTest {
    @Test
    void requestsRollersWhileAutoScoreControlsIntakePosition() {
        assertFalse(IntakeRequest.IDLE.runsRollers());
        assertTrue(IntakeRequest.STOWED.runsRollers());
        assertTrue(IntakeRequest.DEPLOYED.runsRollers());
        assertTrue(IntakeRequest.AUTO_SCORE_SEMI_DEPLOYED.runsRollers());
        assertTrue(IntakeRequest.AUTO_SCORE_SEVENTY_PERCENT_DEPLOYED.runsRollers());
    }

    @Test
    void staysIdleUntilAutoScoreStartsFeeding() {
        AutoScoreIntakeAssist assist = new AutoScoreIntakeAssist();

        IntakeRequest request = assist.update(true, false, false, 10.0);

        assertEquals(IntakeRequest.IDLE, request);
    }

    @Test
    void waitsInitialDelayBeforeOscillatingAfterAutoScoreStartsFeeding() {
        AutoScoreIntakeAssist assist = new AutoScoreIntakeAssist();

        assertEquals(IntakeRequest.IDLE, assist.update(true, true, false, 10.0));
        assertEquals(IntakeRequest.IDLE, assist.update(true, true, false, 10.99));

        assertEquals(
            IntakeRequest.AUTO_SCORE_SEMI_DEPLOYED,
            assist.update(true, true, false, 11.0)
        );
        assertEquals(
            IntakeRequest.AUTO_SCORE_SEMI_DEPLOYED,
            assist.update(true, true, false, 11.99)
        );
        assertEquals(
            IntakeRequest.AUTO_SCORE_SEVENTY_PERCENT_DEPLOYED,
            assist.update(true, true, false, 12.0)
        );
        assertEquals(
            IntakeRequest.AUTO_SCORE_SEVENTY_PERCENT_DEPLOYED,
            assist.update(true, true, false, 12.99)
        );
        assertEquals(
            IntakeRequest.AUTO_SCORE_SEMI_DEPLOYED,
            assist.update(true, true, false, 13.0)
        );
    }

    @Test
    void keepsIntakeDeployedWhenDriverPressesIntakeButtonAfterFeedingStarts() {
        AutoScoreIntakeAssist assist = new AutoScoreIntakeAssist();
        assist.update(true, true, false, 10.0);

        IntakeRequest request = assist.update(true, false, true, 10.1);

        assertEquals(IntakeRequest.DEPLOYED, request);
    }

    @Test
    void retractButtonOverridesAutoScoreUntilItIsReleased() {
        AutoScoreIntakeAssist assist = new AutoScoreIntakeAssist();
        assist.update(true, true, false, false, 10.0);

        assertEquals(
            IntakeRequest.STOWED,
            assist.update(true, true, false, true, 11.0)
        );
        assertEquals(
            IntakeRequest.AUTO_SCORE_SEMI_DEPLOYED,
            assist.update(true, true, false, false, 11.1)
        );
    }

    @Test
    void laterDeployPressOverridesHeldRetractButton() {
        AutoScoreIntakeAssist assist = new AutoScoreIntakeAssist();

        assertEquals(
            IntakeRequest.STOWED,
            assist.update(true, true, false, true, 10.0)
        );
        assertEquals(
            IntakeRequest.DEPLOYED,
            assist.update(true, true, true, true, 10.1)
        );
        assertEquals(
            IntakeRequest.STOWED,
            assist.update(true, true, false, true, 10.2)
        );
    }

    @Test
    void laterRetractPressOverridesHeldDeployButton() {
        AutoScoreIntakeAssist assist = new AutoScoreIntakeAssist();

        assertEquals(
            IntakeRequest.DEPLOYED,
            assist.update(true, true, true, false, 10.0)
        );
        assertEquals(
            IntakeRequest.STOWED,
            assist.update(true, true, true, true, 10.1)
        );
        assertEquals(
            IntakeRequest.DEPLOYED,
            assist.update(true, true, true, false, 10.2)
        );
    }

    @Test
    void returnsToDeployedWhenAutoScoreEndsAfterFeedingStarted() {
        AutoScoreIntakeAssist assist = new AutoScoreIntakeAssist();
        assist.update(true, true, false, 10.0);

        IntakeRequest request = assist.update(false, false, false, 10.1);

        assertEquals(IntakeRequest.DEPLOYED, request);
    }

    @Test
    void onlyRequestsDeployedOnceAfterAutoScoreEnds() {
        AutoScoreIntakeAssist assist = new AutoScoreIntakeAssist();
        assist.update(true, true, false, 10.0);
        assist.update(false, false, false, 10.1);

        IntakeRequest request = assist.update(false, false, false, 10.2);

        assertEquals(IntakeRequest.IDLE, request);
    }
}
