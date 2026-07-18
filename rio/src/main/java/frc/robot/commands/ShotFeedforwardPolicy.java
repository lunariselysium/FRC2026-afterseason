// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.TurretFlywheelConstants;
import frc.robot.scoring.ScoringCalculator.TargetMode;

final class ShotFeedforwardPolicy {
    private ShotFeedforwardPolicy() {}

    static double getAdditionalFeedforwardVolts(TargetMode targetMode) {
        return targetMode == TargetMode.PASS
            ? TurretFlywheelConstants.kPassShotFeedforwardVolts
            : 0.0;
    }
}
