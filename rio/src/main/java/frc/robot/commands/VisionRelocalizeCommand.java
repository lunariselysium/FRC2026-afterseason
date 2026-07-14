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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class VisionRelocalizeCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;
    private final Timer timer = new Timer();
    private final SwerveRequest.RobotCentric stopRequest =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private boolean warningReported;

    public VisionRelocalizeCommand(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        vision.startPoseRelocalization();
        timer.restart();
        warningReported = false;
        SmartDashboard.putString("Auto/BumpRecoveryStatus", "WAITING_FOR_VISION");
    }

    @Override
    public void execute() {
        stopDrivetrain();

        if (!warningReported
            && timer.hasElapsed(BumpCrossingConstants.kRelocalizationWarningSeconds)) {
            warningReported = true;
            DriverStation.reportWarning(
                "Bump recovery is still waiting for a stable multi-tag vision pose; "
                    + "the autonomous routine remains stopped.",
                false
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        stopDrivetrain();
        vision.resumeNormalPoseFusion();
        SmartDashboard.putString(
            "Auto/BumpRecoveryStatus",
            interrupted ? "INTERRUPTED" : "RELOCALIZED"
        );

        if (interrupted && !vision.isPoseRelocalized()) {
            DriverStation.reportWarning(
                "Bump vision recovery was interrupted before pose relocalization completed.",
                false
            );
        }
    }

    @Override
    public boolean isFinished() {
        return vision.isPoseRelocalized();
    }

    private void stopDrivetrain() {
        drivetrain.setControl(
            stopRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        );
    }
}
