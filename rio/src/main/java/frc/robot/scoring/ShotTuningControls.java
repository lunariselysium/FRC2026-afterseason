// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.TurretFlywheelConstants;
import frc.robot.subsystems.Turret;

public class ShotTuningControls {
    public static final String kManualPitchEnabledKey = "ShotTuning/ManualPitchEnabled";
    public static final String kManualFlywheelEnabledKey = "ShotTuning/ManualFlywheelEnabled";
    public static final String kRequestedPitchDegreesKey = "ShotTuning/RequestedPitchDegrees";
    public static final String kRequestedFlywheelRpsKey = "ShotTuning/RequestedFlywheelRps";

    private static final String kManualStatusKey = "ShotTuning/ManualStatus";
    private static final String kManualBlockedKey = "ShotTuning/ManualBlocked";
    private static final String kManualPitchActiveKey = "ShotTuning/ManualPitchActive";
    private static final String kManualFlywheelActiveKey = "ShotTuning/ManualFlywheelActive";
    private static final String kAppliedPitchDegreesKey = "ShotTuning/AppliedPitchDegrees";
    private static final String kAppliedFlywheelRpsKey = "ShotTuning/AppliedFlywheelRps";
    private static final String kAppliedFlywheelRpmKey = "ShotTuning/AppliedFlywheelRpm";

    private final Turret turret;

    private boolean manualFlywheelWasActive;

    public ShotTuningControls(Turret turret) {
        this.turret = turret;
        publishDashboardDefaults();
    }

    public void update(
        boolean normalScoringRequested,
        boolean flywheelButtonRequested,
        boolean publishTelemetry
    ) {
        boolean manualPitchEnabled = SmartDashboard.getBoolean(kManualPitchEnabledKey, false);
        boolean manualFlywheelEnabled = SmartDashboard.getBoolean(kManualFlywheelEnabledKey, false);
        double requestedPitchDegrees = getRequestedPitchDegrees();
        double requestedFlywheelRotationsPerSecond = getRequestedFlywheelRotationsPerSecond();
        boolean manualBlocked = normalScoringRequested || turret.isAnySysIdActive();

        if (publishTelemetry) {
            SmartDashboard.putBoolean(kManualBlockedKey, manualBlocked);
            SmartDashboard.putBoolean(kManualPitchActiveKey, !manualBlocked && manualPitchEnabled);
            SmartDashboard.putBoolean(kManualFlywheelActiveKey, !manualBlocked && manualFlywheelEnabled);
            SmartDashboard.putNumber(kAppliedPitchDegreesKey, requestedPitchDegrees);
            SmartDashboard.putNumber(
                kAppliedFlywheelRpsKey,
                requestedFlywheelRotationsPerSecond
            );
            SmartDashboard.putNumber(
                kAppliedFlywheelRpmKey,
                60.0 * requestedFlywheelRotationsPerSecond
            );
        }

        if (manualBlocked) {
            if (publishTelemetry) {
                SmartDashboard.putString(
                    kManualStatusKey,
                    normalScoringRequested ? "BLOCKED_BY_SCORE" : "BLOCKED_BY_SYSID"
                );
            }
            return;
        }

        if (manualPitchEnabled) {
            turret.setTargetPitchDegrees(requestedPitchDegrees);
        }

        if (manualFlywheelEnabled) {
            turret.runFlywheelAtVelocityRotationsPerSecond(requestedFlywheelRotationsPerSecond);
        } else if (manualFlywheelWasActive && !flywheelButtonRequested) {
            turret.stopFlywheel();
        }

        manualFlywheelWasActive = manualFlywheelEnabled;
        if (publishTelemetry) {
            SmartDashboard.putString(
                kManualStatusKey,
                getManualStatus(manualPitchEnabled, manualFlywheelEnabled)
            );
        }
    }

    public double getRequestedFlywheelRotationsPerSecond() {
        double requestedFlywheelRotationsPerSecond = SmartDashboard.getNumber(
            kRequestedFlywheelRpsKey,
            TurretFlywheelConstants.kTargetVelocityRotationsPerSecond
        );
        double appliedFlywheelRotationsPerSecond = MathUtil.clamp(
            requestedFlywheelRotationsPerSecond,
            ScoringConstants.kMinShotFlywheelRotationsPerSecond,
            ScoringConstants.kMaxShotFlywheelRotationsPerSecond
        );

        return appliedFlywheelRotationsPerSecond;
    }

    private void publishDashboardDefaults() {
        SmartDashboard.putBoolean(kManualPitchEnabledKey, false);
        SmartDashboard.putBoolean(kManualFlywheelEnabledKey, false);
        SmartDashboard.putNumber(kRequestedPitchDegreesKey, getDefaultPitchDegrees());
        SmartDashboard.putNumber(
            kRequestedFlywheelRpsKey,
            TurretFlywheelConstants.kTargetVelocityRotationsPerSecond
        );
        SmartDashboard.putString(kManualStatusKey, "IDLE");
        SmartDashboard.putBoolean(kManualBlockedKey, false);
        SmartDashboard.putBoolean(kManualPitchActiveKey, false);
        SmartDashboard.putBoolean(kManualFlywheelActiveKey, false);
    }

    private double getRequestedPitchDegrees() {
        double requestedPitchDegrees = SmartDashboard.getNumber(
            kRequestedPitchDegreesKey,
            getDefaultPitchDegrees()
        );
        double appliedPitchDegrees = MathUtil.clamp(
            requestedPitchDegrees,
            ScoringConstants.kMinShotPitchDegrees,
            ScoringConstants.kMaxShotPitchDegrees
        );

        return appliedPitchDegrees;
    }

    private String getManualStatus(boolean manualPitchEnabled, boolean manualFlywheelEnabled) {
        if (manualPitchEnabled && manualFlywheelEnabled) {
            return "PITCH_AND_FLYWHEEL";
        }

        if (manualPitchEnabled) {
            return "PITCH";
        }

        if (manualFlywheelEnabled) {
            return "FLYWHEEL";
        }

        return "IDLE";
    }

    private static double getDefaultPitchDegrees() {
        if (ScoringConstants.kHubShotMap.length > 0) {
            return ScoringConstants.kHubShotMap[0].pitchDegrees();
        }

        return (ScoringConstants.kMinShotPitchDegrees
            + ScoringConstants.kMaxShotPitchDegrees) / 2.0;
    }
}
