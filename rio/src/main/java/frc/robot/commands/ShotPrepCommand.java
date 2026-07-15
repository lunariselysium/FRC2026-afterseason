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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScoringConstants;
import frc.robot.scoring.ScoringCalculator;
import frc.robot.scoring.ScoringCalculator.ScoringTarget;
import frc.robot.scoring.ScoringCalculator.TargetSelectionMode;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class ShotPrepCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Turret turret;
    private final Vision vision;
    private final BooleanSupplier scoreRequestedSupplier;

    public ShotPrepCommand(
        CommandSwerveDrivetrain drivetrain,
        Turret turret,
        Vision vision,
        BooleanSupplier scoreRequestedSupplier
    ) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.vision = vision;
        this.scoreRequestedSupplier = scoreRequestedSupplier;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("ShotPrep/Status", "ACTIVE");
        SmartDashboard.putBoolean("ShotPrep/Active", true);
    }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            turret.stopFlywheel();
            SmartDashboard.putString("ShotPrep/Status", "NO_ALLIANCE");
            SmartDashboard.putBoolean("ShotPrep/Active", false);
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

        SmartDashboard.putString("ShotPrep/Status", "ACTIVE");
        SmartDashboard.putBoolean("ShotPrep/Active", true);
        SmartDashboard.putNumber("ShotPrep/TargetHeadingDegrees", target.turretHeadingDegrees());
        SmartDashboard.putNumber("ShotPrep/TargetPitchDegrees", target.shotSetpoint().pitchDegrees());
        SmartDashboard.putNumber(
            "ShotPrep/TargetFlywheelRps",
            target.shotSetpoint().flywheelRotationsPerSecond()
        );
    }

    @Override
    public void end(boolean interrupted) {
        boolean keepFlywheelRunning = ShotHandoffPolicy.shouldKeepFlywheelRunning(
            scoreRequestedSupplier.getAsBoolean(),
            false
        );
        if (!keepFlywheelRunning) {
            turret.stopFlywheel();
        }

        SmartDashboard.putString(
            "ShotPrep/Status",
            interrupted ? "INTERRUPTED" : "ENDED"
        );
        SmartDashboard.putBoolean("ShotPrep/Active", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
