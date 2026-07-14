// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.scoring;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.scoring.AutoScoreIntakeAssist.IntakeRequest;

class AutoScoreIntakeAssistTest {
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
