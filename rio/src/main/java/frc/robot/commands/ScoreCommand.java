// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringConstants;
import frc.robot.scoring.ScoringCalculator;
import frc.robot.scoring.ScoringCalculator.ScoringTarget;
import frc.robot.scoring.ScoringCalculator.TargetSelectionMode;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class ScoreCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Turret turret;
    private final Feeder feeder;
    private final Vision vision;

    private int readyCycles;
    private boolean active;
    private boolean feeding;

    public ScoreCommand(
        CommandSwerveDrivetrain drivetrain,
        Turret turret,
        Feeder feeder,
        Vision vision
    ) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.feeder = feeder;
        this.vision = vision;

        addRequirements(turret, feeder);
    }

    @Override
    public void initialize() {
        active = true;
        feeding = false;
        readyCycles = 0;
        stopFeeding();
        turret.stopShotOutputs();
    }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            readyCycles = 0;
            feeding = false;
            stopFeeding();
            turret.stopShotOutputs();
            publishIdleTelemetry("NO_ALLIANCE");
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

        turret.setTargetHeadingDegrees(target.turretHeadingDegrees());
        turret.setTargetPitchDegrees(target.shotSetpoint().pitchDegrees());
        turret.runFlywheelAtVelocityRotationsPerSecond(
            target.shotSetpoint().flywheelRotationsPerSecond()
        );

        boolean ready = target.shotSetpoint().feedAllowedByDistance()
            && turret.isHeadingAtTarget()
            && turret.isPitchAtTarget()
            && turret.isFlywheelReadyToShoot();
        if (ready) {
            readyCycles++;
        } else {
            readyCycles = 0;
        }

        boolean shouldFeed = readyCycles >= ScoringConstants.kReadyDebounceCycles;
        feeding = shouldFeed;
        if (shouldFeed) {
            feeder.runAll();
            turret.runSerializer();
        } else {
            stopFeeding();
        }

        publishTargetTelemetry(target, hubVisionCorrectionDegrees, ready, shouldFeed);
    }

    @Override
    public void end(boolean interrupted) {
        active = false;
        feeding = false;
        readyCycles = 0;
        stopFeeding();
        turret.stopShotOutputs();
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

    private void publishTargetTelemetry(
        ScoringTarget target,
        OptionalDouble hubVisionCorrectionDegrees,
        boolean ready,
        boolean feeding
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
        SmartDashboard.putNumber("Scoring/PitchDegrees", target.shotSetpoint().pitchDegrees());
        SmartDashboard.putNumber(
            "Scoring/FlywheelRps",
            target.shotSetpoint().flywheelRotationsPerSecond()
        );
        SmartDashboard.putNumber(
            "Scoring/FlywheelFeedingLoadFeedforwardVolts",
            turret.getFlywheelFeedingLoadFeedforwardVolts()
        );
        SmartDashboard.putBoolean(
            "Scoring/FeedAllowedByDistance",
            target.shotSetpoint().feedAllowedByDistance()
        );
        SmartDashboard.putBoolean("Scoring/HeadingReady", turret.isHeadingAtTarget());
        SmartDashboard.putBoolean("Scoring/PitchReady", turret.isPitchAtTarget());
        SmartDashboard.putBoolean("Scoring/FlywheelReady", turret.isFlywheelReadyToShoot());
        SmartDashboard.putBoolean("Scoring/FlywheelAtTarget", turret.isFlywheelAtTarget());
        SmartDashboard.putNumber("Scoring/ReadyCycles", readyCycles);
        SmartDashboard.putBoolean("Scoring/Ready", ready);
        SmartDashboard.putBoolean("Scoring/Feeding", feeding);
    }

    private void publishIdleTelemetry(String status) {
        SmartDashboard.putString("Scoring/Status", status);
        SmartDashboard.putBoolean("Scoring/Ready", false);
        SmartDashboard.putBoolean("Scoring/Feeding", false);
        SmartDashboard.putNumber("Scoring/ReadyCycles", readyCycles);
    }
}
