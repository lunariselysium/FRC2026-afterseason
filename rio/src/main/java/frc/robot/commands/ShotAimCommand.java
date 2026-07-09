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
import frc.robot.scoring.ScoringCalculator;
import frc.robot.scoring.ScoringCalculator.ScoringTarget;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class ShotAimCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Turret turret;
    private final Vision vision;

    public ShotAimCommand(
        CommandSwerveDrivetrain drivetrain,
        Turret turret,
        Vision vision
    ) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.vision = vision;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("ShotTuning/AimCommandStatus", "ACTIVE");
    }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            SmartDashboard.putString("ShotTuning/AimCommandStatus", "NO_ALLIANCE");
            SmartDashboard.putBoolean("ShotTuning/AimActive", false);
            return;
        }

        Pose2d robotPose = drivetrain.getState().Pose;
        OptionalDouble hubYawDegrees = vision.getTurretForwardHubYawDegrees(alliance.get());
        ScoringTarget target = ScoringCalculator.calculateTarget(
            robotPose,
            turret.getHeadingDegrees(),
            alliance.get(),
            hubYawDegrees
        );

        turret.setTargetHeadingDegrees(target.turretHeadingDegrees());
        SmartDashboard.putString("ShotTuning/AimCommandStatus", "ACTIVE");
        SmartDashboard.putBoolean("ShotTuning/AimActive", true);
        SmartDashboard.putBoolean("ShotTuning/AimReady", turret.isHeadingAtTarget());
        SmartDashboard.putNumber(
            "ShotTuning/AimTargetHeadingDegrees",
            target.turretHeadingDegrees()
        );
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString(
            "ShotTuning/AimCommandStatus",
            interrupted ? "INTERRUPTED" : "ENDED"
        );
        SmartDashboard.putBoolean("ShotTuning/AimActive", false);
        SmartDashboard.putBoolean("ShotTuning/AimReady", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
