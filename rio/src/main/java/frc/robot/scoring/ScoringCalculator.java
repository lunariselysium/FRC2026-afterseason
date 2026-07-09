// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.scoring;

import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants.ShotCurve;
import frc.robot.Constants.ScoringConstants.ShotCurveType;
import frc.robot.Constants.ScoringConstants.ShotMapPoint;
import frc.robot.Constants.VisionConstants;

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
        Translation2d turretFieldPoint,
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
        return calculateTarget(robotPose, alliance, hubYawDegrees, OptionalDouble.empty());
    }

    public static ScoringTarget calculateTarget(
        Pose2d robotPose,
        double currentTurretHeadingDegrees,
        Alliance alliance,
        OptionalDouble hubYawDegrees
    ) {
        return calculateTarget(
            robotPose,
            alliance,
            hubYawDegrees,
            OptionalDouble.of(currentTurretHeadingDegrees)
        );
    }

    private static ScoringTarget calculateTarget(
        Pose2d robotPose,
        Alliance alliance,
        OptionalDouble hubYawDegrees,
        OptionalDouble currentTurretHeadingDegrees
    ) {
        boolean hubShot = isInOwnAllianceZone(robotPose.getTranslation(), alliance);
        Translation2d targetPoint = hubShot
            ? getHubCenter(alliance)
            : getNearestPassTarget(robotPose.getTranslation(), alliance);
        Translation2d turretFieldPoint = getTurretFieldPoint(robotPose);
        double distanceMeters = turretFieldPoint.getDistance(targetPoint);
        double fieldBearingDegrees = getFieldBearingDegrees(turretFieldPoint, targetPoint);
        double visualTrimDegrees = hubShot ? getHubVisualTrimDegrees(hubYawDegrees) : 0.0;
        double poseBasedTurretHeadingDegrees = MathUtil.inputModulus(
            fieldBearingDegrees - robotPose.getRotation().getDegrees(),
            -180.0,
            180.0
        );
        double turretHeadingDegrees = MathUtil.inputModulus(
            // Fresh camera yaw is a heading error, so servo from the measured turret heading.
            shouldServoFromVision(hubShot, hubYawDegrees, currentTurretHeadingDegrees)
                ? currentTurretHeadingDegrees.getAsDouble() + visualTrimDegrees
                : poseBasedTurretHeadingDegrees + visualTrimDegrees,
            -180.0,
            180.0
        );
        ShotSetpoint shotSetpoint = evaluateShotCurve(
            distanceMeters,
            hubShot ? ScoringConstants.kHubShotCurve : ScoringConstants.kPassShotCurve,
            !hubShot
        );

        return new ScoringTarget(
            hubShot ? TargetMode.HUB : TargetMode.PASS,
            targetPoint,
            turretFieldPoint,
            distanceMeters,
            fieldBearingDegrees,
            turretHeadingDegrees,
            visualTrimDegrees,
            shotSetpoint
        );
    }

    private static boolean shouldServoFromVision(
        boolean hubShot,
        OptionalDouble hubYawDegrees,
        OptionalDouble currentTurretHeadingDegrees
    ) {
        return hubShot && hubYawDegrees.isPresent() && currentTurretHeadingDegrees.isPresent();
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

    public static ShotSetpoint evaluateShotCurve(
        double distanceMeters,
        ShotCurve shotCurve,
        boolean allowExtrapolation
    ) {
        ShotMapPoint[] shotMap = shotCurve.shotMap();
        if (shotMap.length == 0) {
            return new ShotSetpoint(0.0, 0.0, false);
        }

        boolean inRange = isDistanceInShotMapRange(distanceMeters, shotMap);
        boolean feedAllowedByDistance = inRange || allowExtrapolation;
        double pitchDegrees = evaluateShotValue(
            distanceMeters,
            shotCurve.pitchCurveType(),
            shotMap,
            shotCurve.pitchPolynomialCoefficients(),
            true,
            allowExtrapolation
        );
        double flywheelRotationsPerSecond = evaluateShotValue(
            distanceMeters,
            shotCurve.flywheelCurveType(),
            shotMap,
            shotCurve.flywheelPolynomialCoefficients(),
            false,
            allowExtrapolation
        );

        return new ShotSetpoint(
            clampPitch(pitchDegrees),
            clampFlywheelVelocity(flywheelRotationsPerSecond),
            feedAllowedByDistance
        );
    }

    private static double getFieldBearingDegrees(
        Translation2d robotPosition,
        Translation2d targetPoint
    ) {
        Translation2d robotToTarget = targetPoint.minus(robotPosition);
        return Math.toDegrees(Math.atan2(robotToTarget.getY(), robotToTarget.getX()));
    }

    private static Translation2d getTurretFieldPoint(Pose2d robotPose) {
        Translation3d robotToTurret = VisionConstants.kRobotToTurretYawAxis.getTranslation();
        Translation2d robotToTurret2d = new Translation2d(
            robotToTurret.getX(),
            robotToTurret.getY()
        );

        return robotPose.getTranslation().plus(robotToTurret2d.rotateBy(robotPose.getRotation()));
    }

    private static ShotSetpoint shotFromPoint(ShotMapPoint point, boolean feedAllowedByDistance) {
        return new ShotSetpoint(
            clampPitch(point.pitchDegrees()),
            clampFlywheelVelocity(point.flywheelRotationsPerSecond()),
            feedAllowedByDistance
        );
    }

    private static double evaluateShotValue(
        double distanceMeters,
        ShotCurveType curveType,
        ShotMapPoint[] shotMap,
        double[] polynomialCoefficients,
        boolean pitchValue,
        boolean allowExtrapolation
    ) {
        if (!allowExtrapolation && distanceMeters <= shotMap[0].distanceMeters()) {
            return getShotPointValue(shotMap[0], pitchValue);
        }

        if (!allowExtrapolation
            && distanceMeters >= shotMap[shotMap.length - 1].distanceMeters()) {
            return getShotPointValue(shotMap[shotMap.length - 1], pitchValue);
        }

        if (curveType == ShotCurveType.POLYNOMIAL && polynomialCoefficients.length > 0) {
            return evaluatePolynomial(distanceMeters, polynomialCoefficients);
        }

        return interpolateShotValue(distanceMeters, shotMap, pitchValue);
    }

    private static double interpolateShotValue(
        double distanceMeters,
        ShotMapPoint[] shotMap,
        boolean pitchValue
    ) {
        if (shotMap.length == 1) {
            return getShotPointValue(shotMap[0], pitchValue);
        }

        ShotMapPoint lowerPoint = shotMap[0];
        ShotMapPoint upperPoint = shotMap[1];

        if (distanceMeters >= shotMap[shotMap.length - 1].distanceMeters()) {
            lowerPoint = shotMap[shotMap.length - 2];
            upperPoint = shotMap[shotMap.length - 1];
        } else if (distanceMeters > shotMap[0].distanceMeters()) {
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

        return interpolate(
            getShotPointValue(lowerPoint, pitchValue),
            getShotPointValue(upperPoint, pitchValue),
            interpolation
        );
    }

    private static double evaluatePolynomial(double x, double[] coefficients) {
        double value = 0.0;
        for (int i = coefficients.length - 1; i >= 0; i--) {
            value = value * x + coefficients[i];
        }

        return value;
    }

    private static double getShotPointValue(ShotMapPoint point, boolean pitchValue) {
        return pitchValue ? point.pitchDegrees() : point.flywheelRotationsPerSecond();
    }

    private static boolean isDistanceInShotMapRange(
        double distanceMeters,
        ShotMapPoint[] shotMap
    ) {
        return distanceMeters >= shotMap[0].distanceMeters()
            && distanceMeters <= shotMap[shotMap.length - 1].distanceMeters();
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
