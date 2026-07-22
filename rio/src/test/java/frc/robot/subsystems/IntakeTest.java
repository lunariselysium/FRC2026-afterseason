// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.IntakeConstants;

class IntakeTest {
    @Test
    void manualUnjamTargetMovesTwentyPercentOfTravelOutward() {
        assertEquals(
            14.56,
            Intake.calculateManualUnjamTargetPositionMotorRotations(10.4),
            1.0e-9
        );
    }

    @Test
    void manualUnjamTargetClampsAtDeployedLimit() {
        assertEquals(
            IntakeConstants.kDeployedSetpointMotorRotations,
            Intake.calculateManualUnjamTargetPositionMotorRotations(19.0),
            1.0e-9
        );
    }
}
