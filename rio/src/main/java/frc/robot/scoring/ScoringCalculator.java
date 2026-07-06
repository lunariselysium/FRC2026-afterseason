// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.scoring;

import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants.ShotMapPoint;

public final class ScoringCalculator {
    private ScoringCalculator() {}

    public enum TargetMode {
        HUB,
        PASS
    }

    public record ShotSetpoint(
        double pitchDegrees,
        double flywheelRotationsPerSecond,
        boolean feedAllowedByDistance
    ) {}

    public record ScoringTarget(
        TargetMode mode,
        Translation2d fieldPoint,
        double distanceMeters,
        double fieldBearingDegrees,
        double turretHeadingDegrees,
        double visualTrimDegrees,
        ShotSetpoint shotSetpoint
    ) {}

    public static ScoringTarget calculateTarget(
        Pose2d robotPose,
        Alliance alliance,
        OptionalDouble hubYawDegrees
    ) {
        boolean hubShot = isInOwnAllianceZone(robotPose.getTranslation(), alliance);
        Translation2d targetPoint = hubShot
            ? getHubCenter(alliance)
            : getNearestPassTarget(robotPose.getTranslation(), alliance);
        double distanceMeters = robotPose.getTranslation().getDistance(targetPoint);
        double fieldBearingDegrees = getFieldBearingDegrees(robotPose.getTranslation(), targetPoint);
        double visualTrimDegrees = hubShot ? getHubVisualTrimDegrees(hubYawDegrees) : 0.0;
        double turretHeadingDegrees = MathUtil.inputModulus(
            fieldBearingDegrees - robotPose.getRotation().getDegrees() + visualTrimDegrees,
            -180.0,
            180.0
        );
        ShotSetpoint shotSetpoint = interpolateShotSetpoint(
            distanceMeters,
            hubShot ? ScoringConstants.kHubShotMap : ScoringConstants.kPassShotMap,
            !hubShot
        );

        return new ScoringTarget(
            hubShot ? TargetMode.HUB : TargetMode.PASS,
            targetPoint,
            distanceMeters,
            fieldBearingDegrees,
            turretHeadingDegrees,
            visualTrimDegrees,
            shotSetpoint
        );
    }

    public static boolean isInOwnAllianceZone(Translation2d robotPosition, Alliance alliance) {
        if (alliance == Alliance.Red) {
            return robotPosition.getX()
                >= ScoringConstants.kFieldLengthMeters - ScoringConstants.kAllianceZoneDepthMeters;
        }

        return robotPosition.getX() <= ScoringConstants.kAllianceZoneDepthMeters;
    }

    public static Translation2d getHubCenter(Alliance alliance) {
        return alliance == Alliance.Red
            ? ScoringConstants.kRedHubCenterMeters
            : ScoringConstants.kBlueHubCenterMeters;
    }

    public static Translation2d getNearestPassTarget(Translation2d robotPosition, Alliance alliance) {
        Translation2d lowerTarget = alliance == Alliance.Red
            ? ScoringConstants.kRedLowerPassTargetMeters
            : ScoringConstants.kBlueLowerPassTargetMeters;
        Translation2d upperTarget = alliance == Alliance.Red
            ? ScoringConstants.kRedUpperPassTargetMeters
            : ScoringConstants.kBlueUpperPassTargetMeters;

        return robotPosition.getDistance(lowerTarget) <= robotPosition.getDistance(upperTarget)
            ? lowerTarget
            : upperTarget;
    }

    public static boolean isHubTagForAlliance(int tagId, Alliance alliance) {
        int[] hubTagIds = alliance == Alliance.Red
            ? ScoringConstants.kRedHubTagIds
            : ScoringConstants.kBlueHubTagIds;
        for (int hubTagId : hubTagIds) {
            if (hubTagId == tagId) {
                return true;
            }
        }

        return false;
    }

    public static double getHubVisualTrimDegrees(OptionalDouble hubYawDegrees) {
        if (hubYawDegrees.isEmpty()) {
            return 0.0;
        }

        double yawDegrees = hubYawDegrees.getAsDouble();
        if (Math.abs(yawDegrees) <= ScoringConstants.kHubVisualTrimToleranceDegrees) {
            return 0.0;
        }

        return MathUtil.clamp(
            ScoringConstants.kHubVisualTrimYawSign
                * ScoringConstants.kHubVisualTrimYawGain
                * yawDegrees,
            -ScoringConstants.kHubVisualTrimMaxCorrectionDegrees,
            ScoringConstants.kHubVisualTrimMaxCorrectionDegrees
        );
    }

    public static ShotSetpoint interpolateShotSetpoint(
        double distanceMeters,
        ShotMapPoint[] shotMap,
        boolean allowExtrapolation
    ) {
        if (shotMap.length == 0) {
            return new ShotSetpoint(0.0, 0.0, false);
        }

        if (shotMap.length == 1) {
            ShotMapPoint point = shotMap[0];
            return new ShotSetpoint(
                clampPitch(point.pitchDegrees()),
                clampFlywheelVelocity(point.flywheelRotationsPerSecond()),
                allowExtrapolation
            );
        }

        boolean inRange = distanceMeters >= shotMap[0].distanceMeters()
            && distanceMeters <= shotMap[shotMap.length - 1].distanceMeters();
        ShotMapPoint lowerPoint = shotMap[0];
        ShotMapPoint upperPoint = shotMap[1];

        if (distanceMeters <= shotMap[0].distanceMeters()) {
            if (!allowExtrapolation) {
                return shotFromPoint(shotMap[0], inRange);
            }
        } else if (distanceMeters >= shotMap[shotMap.length - 1].distanceMeters()) {
            lowerPoint = shotMap[shotMap.length - 2];
            upperPoint = shotMap[shotMap.length - 1];
            if (!allowExtrapolation) {
                return shotFromPoint(shotMap[shotMap.length - 1], inRange);
            }
        } else {
            for (int i = 0; i < shotMap.length - 1; i++) {
                if (distanceMeters >= shotMap[i].distanceMeters()
                    && distanceMeters <= shotMap[i + 1].distanceMeters()) {
                    lowerPoint = shotMap[i];
                    upperPoint = shotMap[i + 1];
                    break;
                }
            }
        }

        double distanceRangeMeters = upperPoint.distanceMeters() - lowerPoint.distanceMeters();
        double interpolation = distanceRangeMeters == 0.0
            ? 0.0
            : (distanceMeters - lowerPoint.distanceMeters()) / distanceRangeMeters;
        double pitchDegrees = interpolate(
            lowerPoint.pitchDegrees(),
            upperPoint.pitchDegrees(),
            interpolation
        );
        double flywheelRotationsPerSecond = interpolate(
            lowerPoint.flywheelRotationsPerSecond(),
            upperPoint.flywheelRotationsPerSecond(),
            interpolation
        );

        return new ShotSetpoint(
            clampPitch(pitchDegrees),
            clampFlywheelVelocity(flywheelRotationsPerSecond),
            inRange || allowExtrapolation
        );
    }

    private static double getFieldBearingDegrees(
        Translation2d robotPosition,
        Translation2d targetPoint
    ) {
        Translation2d robotToTarget = targetPoint.minus(robotPosition);
        return Math.toDegrees(Math.atan2(robotToTarget.getY(), robotToTarget.getX()));
    }

    private static ShotSetpoint shotFromPoint(ShotMapPoint point, boolean feedAllowedByDistance) {
        return new ShotSetpoint(
            clampPitch(point.pitchDegrees()),
            clampFlywheelVelocity(point.flywheelRotationsPerSecond()),
            feedAllowedByDistance
        );
    }

    private static double interpolate(double start, double end, double t) {
        return start + (end - start) * t;
    }

    private static double clampPitch(double pitchDegrees) {
        return MathUtil.clamp(
            pitchDegrees,
            ScoringConstants.kMinShotPitchDegrees,
            ScoringConstants.kMaxShotPitchDegrees
        );
    }

    private static double clampFlywheelVelocity(double flywheelRotationsPerSecond) {
        return MathUtil.clamp(
            flywheelRotationsPerSecond,
            ScoringConstants.kMinShotFlywheelRotationsPerSecond,
            ScoringConstants.kMaxShotFlywheelRotationsPerSecond
        );
    }
}
