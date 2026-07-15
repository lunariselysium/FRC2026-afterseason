// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class FeedControlStateMachineTest {
    @Test
    void runsReducedFeedForFiveOutOfRangeCyclesBeforePausing() {
        FeedControlStateMachine controller = new FeedControlStateMachine(1, 5, 2);

        assertEquals(FeedControlStateMachine.OutputMode.FULL, controller.update(true, true));

        for (int cycle = 0; cycle < 5; cycle++) {
            assertEquals(
                FeedControlStateMachine.OutputMode.REDUCED,
                controller.update(true, false)
            );
        }

        assertEquals(FeedControlStateMachine.OutputMode.STOPPED, controller.update(true, false));
    }

    @Test
    void resumesFullFeedAfterTwoGoodFlywheelCyclesFollowingAPause() {
        FeedControlStateMachine controller = new FeedControlStateMachine(1, 5, 2);

        assertEquals(FeedControlStateMachine.OutputMode.FULL, controller.update(true, true));
        for (int cycle = 0; cycle < 5; cycle++) {
            controller.update(true, false);
        }
        assertEquals(FeedControlStateMachine.OutputMode.STOPPED, controller.update(true, false));

        assertEquals(FeedControlStateMachine.OutputMode.STOPPED, controller.update(true, true));
        assertEquals(FeedControlStateMachine.OutputMode.FULL, controller.update(true, true));
    }

    @Test
    void returnsToFullFeedImmediatelyWhenFlywheelRecoversDuringReducedFeed() {
        FeedControlStateMachine controller = new FeedControlStateMachine(1, 5, 2);

        assertEquals(FeedControlStateMachine.OutputMode.FULL, controller.update(true, true));
        assertEquals(FeedControlStateMachine.OutputMode.REDUCED, controller.update(true, false));
        assertEquals(FeedControlStateMachine.OutputMode.FULL, controller.update(true, true));
    }

    @Test
    void stopsImmediatelyWhenANonFlywheelInterlockIsLost() {
        FeedControlStateMachine controller = new FeedControlStateMachine(1, 5, 2);

        assertEquals(FeedControlStateMachine.OutputMode.FULL, controller.update(true, true));
        assertEquals(FeedControlStateMachine.OutputMode.STOPPED, controller.update(false, true));
    }
}
