// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.scoring;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ScoringConstants;
import frc.robot.scoring.ScoringCalculator.ScoringTarget;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class ScoringTelemetry {
    private static final String kDashboardPrefix = "ShotTuning/";

    private final CommandSwerveDrivetrain drivetrain;
    private final Turret turret;
    private final Vision vision;
    private final Field2d shotField = new Field2d();

    public ScoringTelemetry(
        CommandSwerveDrivetrain drivetrain,
        Turret turret,
        Vision vision
    ) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.vision = vision;
        SmartDashboard.putData(kDashboardPrefix + "Field", shotField);
    }

    public void update() {
        var drivetrainState = drivetrain.getState();
        Pose2d robotPose = drivetrainState.Pose;
        ChassisSpeeds robotSpeeds = drivetrainState.Speeds;
        Pose2d predictedRobotPose = ScoringCalculator.predictRobotPose(
            robotPose,
            robotSpeeds,
            ScoringConstants.kShotMotionPredictionSeconds
        );
        double robotHeadingDegrees = robotPose.getRotation().getDegrees();
        Optional<Alliance> alliance = DriverStation.getAlliance();

        shotField.setRobotPose(robotPose);
        SmartDashboard.putNumber(kDashboardPrefix + "RobotX", robotPose.getX());
        SmartDashboard.putNumber(kDashboardPrefix + "RobotY", robotPose.getY());
        SmartDashboard.putNumber(
            kDashboardPrefix + "RobotHeadingDegrees",
            wrapDegrees(robotHeadingDegrees)
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "RobotHeadingRawDegrees",
            robotHeadingDegrees
        );
        SmartDashboard.putBoolean(kDashboardPrefix + "AllianceKnown", alliance.isPresent());
        SmartDashboard.putString(
            kDashboardPrefix + "Alliance",
            alliance.isPresent() ? alliance.get().name() : "UNKNOWN"
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "MotionPredictionSeconds",
            ScoringConstants.kShotMotionPredictionSeconds
        );
        SmartDashboard.putNumber(kDashboardPrefix + "PredictedRobotX", predictedRobotPose.getX());
        SmartDashboard.putNumber(kDashboardPrefix + "PredictedRobotY", predictedRobotPose.getY());
        SmartDashboard.putNumber(
            kDashboardPrefix + "PredictedRobotHeadingDegrees",
            wrapDegrees(predictedRobotPose.getRotation().getDegrees())
        );

        if (alliance.isEmpty()) {
            SmartDashboard.putBoolean(kDashboardPrefix + "TargetValid", false);
            SmartDashboard.putString(kDashboardPrefix + "Status", "NO_ALLIANCE");
            clearFieldTargetObjects();
            publishTurretState();
            return;
        }

        OptionalDouble hubVisionCorrectionDegrees =
            vision.getTurretForwardHubVisionCorrectionDegrees(alliance.get());
        ScoringTarget target = ScoringCalculator.calculateTarget(
            predictedRobotPose,
            robotSpeeds,
            ScoringConstants.kShotTimeOfFlightSeconds,
            alliance.get(),
            hubVisionCorrectionDegrees
        );

        SmartDashboard.putBoolean(kDashboardPrefix + "TargetValid", true);
        SmartDashboard.putString(kDashboardPrefix + "Status", "READY_TO_TUNE");
        publishTarget(target, hubVisionCorrectionDegrees);
        publishTurretState();
    }

    private void publishTarget(
        ScoringTarget target,
        OptionalDouble hubVisionCorrectionDegrees
    ) {
        SmartDashboard.putString(kDashboardPrefix + "TargetMode", target.mode().name());
        SmartDashboard.putNumber(kDashboardPrefix + "TargetX", target.fieldPoint().getX());
        SmartDashboard.putNumber(kDashboardPrefix + "TargetY", target.fieldPoint().getY());
        SmartDashboard.putNumber(
            kDashboardPrefix + "CompensatedTargetX",
            target.compensatedFieldPoint().getX()
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "CompensatedTargetY",
            target.compensatedFieldPoint().getY()
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "ShotTimeOfFlightSeconds",
            ScoringConstants.kShotTimeOfFlightSeconds
        );
        SmartDashboard.putNumber(kDashboardPrefix + "TurretFieldX", target.turretFieldPoint().getX());
        SmartDashboard.putNumber(kDashboardPrefix + "TurretFieldY", target.turretFieldPoint().getY());
        SmartDashboard.putNumber(kDashboardPrefix + "DistanceMeters", target.distanceMeters());
        SmartDashboard.putNumber(
            kDashboardPrefix + "DistanceFeet",
            Units.metersToFeet(target.distanceMeters())
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "FieldBearingDegrees",
            target.fieldBearingDegrees()
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "SuggestedTurretHeadingDegrees",
            target.turretHeadingDegrees()
        );
        SmartDashboard.putNumber(kDashboardPrefix + "VisualTrimDegrees", target.visualTrimDegrees());
        SmartDashboard.putBoolean(
            kDashboardPrefix + "HubVisionAssistAvailable",
            hubVisionCorrectionDegrees.isPresent()
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "HubVisionCorrectionDegrees",
            hubVisionCorrectionDegrees.isPresent()
                ? hubVisionCorrectionDegrees.getAsDouble()
                : 0.0
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "SuggestedPitchDegrees",
            target.shotSetpoint().pitchDegrees()
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "SuggestedFlywheelRps",
            target.shotSetpoint().flywheelRotationsPerSecond()
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "SuggestedFlywheelRpm",
            60.0 * target.shotSetpoint().flywheelRotationsPerSecond()
        );
        SmartDashboard.putBoolean(
            kDashboardPrefix + "SuggestedFeedAllowedByDistance",
            target.shotSetpoint().feedAllowedByDistance()
        );
        publishFieldTargetObjects(target);
    }

    private void publishTurretState() {
        SmartDashboard.putNumber(kDashboardPrefix + "MeasuredHeadingDegrees", turret.getHeadingDegrees());
        SmartDashboard.putNumber(
            kDashboardPrefix + "TargetHeadingDegrees",
            turret.getTargetHeadingDegrees()
        );
        SmartDashboard.putNumber(kDashboardPrefix + "MeasuredPitchDegrees", turret.getPitchDegrees());
        SmartDashboard.putNumber(
            kDashboardPrefix + "TargetPitchDegrees",
            turret.getTargetPitchDegrees()
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "MeasuredFlywheelRps",
            turret.getFlywheelVelocityRotationsPerSecond()
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "MeasuredFlywheelRpm",
            60.0 * turret.getFlywheelVelocityRotationsPerSecond()
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "TargetFlywheelRps",
            turret.getTargetFlywheelVelocityRotationsPerSecond()
        );
        SmartDashboard.putNumber(
            kDashboardPrefix + "FlywheelFeedingLoadFeedforwardVolts",
            turret.getFlywheelFeedingLoadFeedforwardVolts()
        );
        SmartDashboard.putBoolean(kDashboardPrefix + "HeadingReady", turret.isHeadingAtTarget());
        SmartDashboard.putBoolean(kDashboardPrefix + "PitchReady", turret.isPitchAtTarget());
        SmartDashboard.putBoolean(kDashboardPrefix + "FlywheelReady", turret.isFlywheelReadyToShoot());
        SmartDashboard.putBoolean(kDashboardPrefix + "FlywheelAtTarget", turret.isFlywheelAtTarget());
        SmartDashboard.putBoolean(kDashboardPrefix + "PitchHomed", turret.isPitchHomed());
    }

    private static double wrapDegrees(double degrees) {
        return MathUtil.inputModulus(degrees, -180.0, 180.0);
    }

    private void publishFieldTargetObjects(ScoringTarget target) {
        Pose2d targetPose = new Pose2d(target.fieldPoint(), Rotation2d.kZero);
        Pose2d compensatedTargetPose =
            new Pose2d(target.compensatedFieldPoint(), Rotation2d.kZero);
        Pose2d turretPose = new Pose2d(target.turretFieldPoint(), Rotation2d.kZero);

        shotField.getObject("Target").setPose(targetPose);
        shotField.getObject("CompensatedTarget").setPose(compensatedTargetPose);
        shotField.getObject("Turret").setPose(turretPose);
        shotField.getObject("AimLine").setPoses(turretPose, compensatedTargetPose);
    }

    private void clearFieldTargetObjects() {
        shotField.getObject("Target").setPoses();
        shotField.getObject("CompensatedTarget").setPoses();
        shotField.getObject("Turret").setPoses();
        shotField.getObject("AimLine").setPoses();
    }
}
