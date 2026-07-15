// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShotHandoffPolicyTest {
    @Test
    void keepsFlywheelRunningWhileEitherShotControlIsStillHeld() {
        assertFalse(ShotHandoffPolicy.shouldKeepFlywheelRunning(false, false));
        assertTrue(ShotHandoffPolicy.shouldKeepFlywheelRunning(true, false));
        assertTrue(ShotHandoffPolicy.shouldKeepFlywheelRunning(false, true));
    }
}
