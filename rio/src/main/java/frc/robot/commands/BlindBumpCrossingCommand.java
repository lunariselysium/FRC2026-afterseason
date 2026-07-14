// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BumpCrossingConstants;
import frc.robot.autonomous.BumpCrossingDirection;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class BlindBumpCrossingCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final BumpCrossingDirection direction;
    private final Timer timer = new Timer();
    private final SwerveRequest.RobotCentric robotCentricRequest =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private boolean maximumDriveTimeWarningReported;

    public BlindBumpCrossingCommand(
        CommandSwerveDrivetrain drivetrain,
        Vision vision,
        BumpCrossingDirection direction
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.direction = direction;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        vision.suppressPoseFusionForBlindCrossing();
        timer.restart();
        maximumDriveTimeWarningReported = false;

        SmartDashboard.putBoolean("Auto/BumpCrossingActive", true);
        SmartDashboard.putString("Auto/BumpCrossingDirection", direction.name());
        SmartDashboard.putNumber(
            "Auto/BumpCrossingSpeedMetersPerSecond",
            BumpCrossingConstants.kCrossingSpeedMetersPerSecond
        );

        if (!DriverStation.isAutonomousEnabled()) {
            DriverStation.reportWarning(
                "Blind bump crossing was scheduled outside enabled autonomous; "
                    + "the drivetrain will remain stopped.",
                false
            );
        }
        if (BumpCrossingConstants.kCrossingSpeedMetersPerSecond <= 0.0) {
            DriverStation.reportWarning(
                "Bump crossing speed is zero; set "
                    + "BumpCrossingConstants.kCrossingSpeedMetersPerSecond before use.",
                false
            );
        }
        if (BumpCrossingConstants.kCrossingDriveTimeSeconds <= 0.0) {
            DriverStation.reportWarning(
                "Bump crossing drive time is zero; set "
                    + "BumpCrossingConstants.kCrossingDriveTimeSeconds before use.",
                false
            );
        } else if (BumpCrossingConstants.kCrossingDriveTimeSeconds
            > BumpCrossingConstants.kMaximumDriveTimeSeconds) {
            DriverStation.reportWarning(
                "Bump crossing drive time exceeds the safety maximum and will be clamped to "
                    + BumpCrossingConstants.kMaximumDriveTimeSeconds
                    + " seconds.",
                false
            );
        }
    }

    @Override
    public void execute() {
        if (!DriverStation.isAutonomousEnabled()) {
            stopDrivetrain();
            return;
        }

        if (timer.hasElapsed(BumpCrossingConstants.kMaximumDriveTimeSeconds)) {
            stopDrivetrain();
            if (!maximumDriveTimeWarningReported) {
                maximumDriveTimeWarningReported = true;
                DriverStation.reportWarning(
                    "Blind bump crossing reached its maximum drive time and was stopped. "
                        + "Vision recovery will begin next.",
                    false
                );
            }
            return;
        }

        double speedMetersPerSecond = BumpCrossingConstants.kCrossingSpeedMetersPerSecond;
        drivetrain.setControl(
            robotCentricRequest
                .withVelocityX(direction.getVelocityXMetersPerSecond(speedMetersPerSecond))
                .withVelocityY(direction.getVelocityYMetersPerSecond(speedMetersPerSecond))
                .withRotationalRate(0.0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        stopDrivetrain();
        SmartDashboard.putBoolean("Auto/BumpCrossingActive", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void stopDrivetrain() {
        drivetrain.setControl(
            robotCentricRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        );
    }
}
