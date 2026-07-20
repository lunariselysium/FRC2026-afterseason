// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringConstants;
import frc.robot.TelemetryRateLimiter;
import frc.robot.scoring.ScoringCalculator;
import frc.robot.scoring.ScoringCalculator.ShotSetpoint;
import frc.robot.scoring.ScoringCalculator.ScoringTarget;
import frc.robot.scoring.ScoringCalculator.TargetMode;
import frc.robot.scoring.ScoringCalculator.TargetSelectionMode;
import frc.robot.scoring.ShotDistanceTuning;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class ScoreCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Turret turret;
    private final Feeder feeder;
    private final Vision vision;
    private final BooleanSupplier shotPrepRequestedSupplier;
    private final ShotDistanceTuning shotDistanceTuning;
    private final TelemetryRateLimiter telemetryRateLimiter =
        TelemetryRateLimiter.forRobotTelemetryPhase(4);
    private final FeedControlStateMachine feedController = new FeedControlStateMachine(
        ScoringConstants.kReadyDebounceCycles,
        ScoringConstants.kFlywheelReducedFeedCycles,
        ScoringConstants.kFlywheelResumeReadyCycles
    );

    private boolean active;
    private boolean feeding;

    public ScoreCommand(
        CommandSwerveDrivetrain drivetrain,
        Turret turret,
        Feeder feeder,
        Vision vision,
        BooleanSupplier shotPrepRequestedSupplier,
        ShotDistanceTuning shotDistanceTuning
    ) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.feeder = feeder;
        this.vision = vision;
        this.shotPrepRequestedSupplier = shotPrepRequestedSupplier;
        this.shotDistanceTuning = shotDistanceTuning;

        addRequirements(turret, feeder);
    }

    @Override
    public void initialize() {
        active = true;
        feeding = false;
        feedController.reset();
        drivetrain.useShootingDriveCurrentLimit();
        stopFeeding();
    }

    @Override
    public void execute() {
        boolean publishTelemetry = telemetryRateLimiter.shouldPublish(
            Timer.getFPGATimestamp()
        );
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            feedController.reset();
            feeding = false;
            stopFeeding();
            turret.stopShotOutputs();
            if (publishTelemetry) {
                publishIdleTelemetry("NO_ALLIANCE");
            }
            return;
        }

        var drivetrainState = drivetrain.getState();
        Pose2d robotPose = ScoringCalculator.predictRobotPose(
            drivetrainState.Pose,
            drivetrainState.Speeds,
            ScoringConstants.kShotMotionPredictionSeconds
        );
        OptionalDouble hubVisionCorrectionDegrees =
            vision.getTurretForwardHubVisionCorrectionDegrees(alliance.get());
        ScoringTarget target = ScoringCalculator.calculateTarget(
            robotPose,
            drivetrainState.Speeds,
            ScoringConstants.kShotTimeOfFlightSeconds,
            alliance.get(),
            hubVisionCorrectionDegrees,
            DriverStation.isAutonomousEnabled()
                ? TargetSelectionMode.HUB_ONLY
                : TargetSelectionMode.AUTOMATIC
        );
        ShotSetpoint shotSetpoint = shotDistanceTuning.evaluateShotSetpoint(
            target.distanceMeters(),
            target.mode() == TargetMode.HUB
                ? ScoringConstants.kHubShotCurve
                : ScoringConstants.kPassShotCurve,
            target.mode() == TargetMode.PASS
        );

        turret.setTargetHeadingDegrees(target.turretHeadingDegrees());
        turret.setTargetPitchDegrees(shotSetpoint.pitchDegrees());
        turret.runFlywheelAtVelocityRotationsPerSecond(
            shotSetpoint.flywheelRotationsPerSecond(),
            ShotFeedforwardPolicy.getAdditionalFeedforwardVolts(target.mode())
        );

        boolean feedInterlocksReady = shotSetpoint.feedAllowedByDistance()
            && turret.isHeadingAtTarget()
            && turret.isPitchAtTarget();
        boolean flywheelReady = turret.isFlywheelReadyToShoot();
        boolean ready = feedInterlocksReady && flywheelReady;
        FeedControlStateMachine.OutputMode feedMode = feedController.update(
            feedInterlocksReady,
            flywheelReady
        );
        feeding = feedMode != FeedControlStateMachine.OutputMode.STOPPED;
        applyFeedMode(feedMode);

        if (publishTelemetry) {
            publishTargetTelemetry(
                target,
                shotSetpoint,
                hubVisionCorrectionDegrees,
                ready,
                feedMode
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        active = false;
        feeding = false;
        feedController.reset();
        stopFeeding();
        if (!ShotHandoffPolicy.shouldKeepFlywheelRunning(
            false,
            shotPrepRequestedSupplier.getAsBoolean()
        )) {
            turret.stopFlywheel();
        }
        drivetrain.useNormalDriveCurrentLimit();
        publishIdleTelemetry(interrupted ? "INTERRUPTED" : "ENDED");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean isActive() {
        return active;
    }

    public boolean isFeeding() {
        return feeding;
    }

    private void stopFeeding() {
        feeder.stopAll();
        turret.stopSerializer();
    }

    private void applyFeedMode(FeedControlStateMachine.OutputMode feedMode) {
        switch (feedMode) {
            case FULL -> {
                feeder.runAll();
                turret.runSerializer();
            }
            case REDUCED -> {
                feeder.runAllWithUpperFeedScale(ScoringConstants.kReducedFeedOutputScale);
                turret.runSerializerAtScale(ScoringConstants.kReducedFeedOutputScale);
            }
            case STOPPED -> stopFeeding();
        }
    }

    private void publishTargetTelemetry(
        ScoringTarget target,
        ShotSetpoint shotSetpoint,
        OptionalDouble hubVisionCorrectionDegrees,
        boolean ready,
        FeedControlStateMachine.OutputMode feedMode
    ) {
        SmartDashboard.putString("Scoring/Status", "ACTIVE");
        SmartDashboard.putString("Scoring/TargetMode", target.mode().name());
        SmartDashboard.putNumber("Scoring/TargetX", target.fieldPoint().getX());
        SmartDashboard.putNumber("Scoring/TargetY", target.fieldPoint().getY());
        SmartDashboard.putNumber(
            "Scoring/CompensatedTargetX",
            target.compensatedFieldPoint().getX()
        );
        SmartDashboard.putNumber(
            "Scoring/CompensatedTargetY",
            target.compensatedFieldPoint().getY()
        );
        SmartDashboard.putNumber(
            "Scoring/ShotTimeOfFlightSeconds",
            ScoringConstants.kShotTimeOfFlightSeconds
        );
        SmartDashboard.putNumber("Scoring/TurretFieldX", target.turretFieldPoint().getX());
        SmartDashboard.putNumber("Scoring/TurretFieldY", target.turretFieldPoint().getY());
        SmartDashboard.putNumber("Scoring/DistanceMeters", target.distanceMeters());
        SmartDashboard.putNumber(
            "Scoring/InterpolationDistanceMeters",
            shotDistanceTuning.getBelievedDistanceMeters(target.distanceMeters())
        );
        SmartDashboard.putNumber("Scoring/FieldBearingDegrees", target.fieldBearingDegrees());
        SmartDashboard.putNumber("Scoring/TurretHeadingDegrees", target.turretHeadingDegrees());
        SmartDashboard.putNumber("Scoring/VisualTrimDegrees", target.visualTrimDegrees());
        SmartDashboard.putBoolean(
            "Scoring/HubVisionAssistAvailable",
            hubVisionCorrectionDegrees.isPresent()
        );
        SmartDashboard.putNumber(
            "Scoring/HubVisionCorrectionDegrees",
            hubVisionCorrectionDegrees.isPresent()
                ? hubVisionCorrectionDegrees.getAsDouble()
                : 0.0
        );
        SmartDashboard.putNumber("Scoring/PitchDegrees", shotSetpoint.pitchDegrees());
        SmartDashboard.putNumber(
            "Scoring/FlywheelRps",
            shotSetpoint.flywheelRotationsPerSecond()
        );
        SmartDashboard.putNumber(
            "Scoring/FlywheelFeedingLoadFeedforwardVolts",
            turret.getFlywheelFeedingLoadFeedforwardVolts()
        );
        SmartDashboard.putNumber(
            "Scoring/FlywheelShotFeedforwardVolts",
            turret.getFlywheelShotFeedforwardVolts()
        );
        SmartDashboard.putBoolean(
            "Scoring/FeedAllowedByDistance",
            shotSetpoint.feedAllowedByDistance()
        );
        SmartDashboard.putBoolean("Scoring/HeadingReady", turret.isHeadingAtTarget());
        SmartDashboard.putBoolean("Scoring/PitchReady", turret.isPitchAtTarget());
        SmartDashboard.putBoolean("Scoring/FlywheelReady", turret.isFlywheelReadyToShoot());
        SmartDashboard.putBoolean("Scoring/FlywheelAtTarget", turret.isFlywheelAtTarget());
        SmartDashboard.putNumber("Scoring/ReadyCycles", feedController.getGoodCycles());
        SmartDashboard.putBoolean("Scoring/Ready", ready);
        SmartDashboard.putString("Scoring/FeedMode", feedMode.name());
        SmartDashboard.putNumber(
            "Scoring/FlywheelOutOfRangeCycles",
            feedController.getOutOfRangeCycles()
        );
        SmartDashboard.putBoolean("Scoring/Feeding", feeding);
    }

    private void publishIdleTelemetry(String status) {
        SmartDashboard.putString("Scoring/Status", status);
        SmartDashboard.putBoolean("Scoring/Ready", false);
        SmartDashboard.putBoolean("Scoring/Feeding", false);
        SmartDashboard.putString("Scoring/FeedMode", FeedControlStateMachine.OutputMode.STOPPED.name());
        SmartDashboard.putNumber("Scoring/ReadyCycles", feedController.getGoodCycles());
        SmartDashboard.putNumber("Scoring/FlywheelOutOfRangeCycles", 0.0);
    }
}
