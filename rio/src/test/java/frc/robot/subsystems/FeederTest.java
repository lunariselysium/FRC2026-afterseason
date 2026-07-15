// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.FeederConstants;

class FeederTest {
    @Test
    void reducedUpperFeedKeepsFloorFullSpeedWhileScalingBeltAndHandoff() {
        Feeder.TargetOutputs outputs = Feeder.calculateUpperFeedScaledOutputs(0.60);

        assertEquals(FeederConstants.kFloorMotorOutput, outputs.floorOutput());
        assertEquals(FeederConstants.kBeltMotorOutput * 0.60, outputs.beltOutput());
        assertEquals(FeederConstants.kHandoffWheelMotorOutput * 0.60, outputs.handoffWheelOutput());
    }

    @Test
    void reverseFeedRunsEveryFeederStageBackward() {
        Feeder.TargetOutputs outputs = Feeder.calculateReversedOutputs();

        assertEquals(-FeederConstants.kFloorMotorOutput, outputs.floorOutput());
        assertEquals(-FeederConstants.kBeltMotorOutput, outputs.beltOutput());
        assertEquals(-FeederConstants.kHandoffWheelMotorOutput, outputs.handoffWheelOutput());
    }
}
