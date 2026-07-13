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

        IntakeRequest request = assist.update(true, false, false);

        assertEquals(IntakeRequest.IDLE, request);
    }

    @Test
    void retractsAfterAutoScoreStartsFeedingWhenIntakeButtonIsNotPressed() {
        AutoScoreIntakeAssist assist = new AutoScoreIntakeAssist();

        IntakeRequest request = assist.update(true, true, false);

        assertEquals(IntakeRequest.AUTO_SCORE_RETRACTED, request);
    }

    @Test
    void keepsIntakeDeployedWhenDriverPressesIntakeButtonAfterFeedingStarts() {
        AutoScoreIntakeAssist assist = new AutoScoreIntakeAssist();
        assist.update(true, true, false);

        IntakeRequest request = assist.update(true, false, true);

        assertEquals(IntakeRequest.DEPLOYED, request);
    }

    @Test
    void returnsToDeployedWhenAutoScoreEndsAfterFeedingStarted() {
        AutoScoreIntakeAssist assist = new AutoScoreIntakeAssist();
        assist.update(true, true, false);

        IntakeRequest request = assist.update(false, false, false);

        assertEquals(IntakeRequest.DEPLOYED, request);
    }

    @Test
    void onlyRequestsDeployedOnceAfterAutoScoreEnds() {
        AutoScoreIntakeAssist assist = new AutoScoreIntakeAssist();
        assist.update(true, true, false);
        assist.update(false, false, false);

        IntakeRequest request = assist.update(false, false, false);

        assertEquals(IntakeRequest.IDLE, request);
    }
}
