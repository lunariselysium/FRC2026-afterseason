// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.TurretFlywheelConstants;
import frc.robot.scoring.ScoringCalculator.TargetMode;

class ShotFeedforwardPolicyTest {
    @Test
    void addsExtraFeedforwardForPassShotsOnly() {
        assertEquals(
            TurretFlywheelConstants.kPassShotFeedforwardVolts,
            ShotFeedforwardPolicy.getAdditionalFeedforwardVolts(TargetMode.PASS),
            1.0e-9
        );
        assertEquals(
            0.0,
            ShotFeedforwardPolicy.getAdditionalFeedforwardVolts(TargetMode.HUB),
            1.0e-9
        );
    }
}
