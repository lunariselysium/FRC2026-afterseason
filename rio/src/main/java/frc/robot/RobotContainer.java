// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private final double maxSpeedMetersPerSecond =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRateRadiansPerSecond =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeedMetersPerSecond * 0.1)
            .withRotationalDeadband(maxAngularRateRadiansPerSecond * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(maxSpeedMetersPerSecond);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Turret turret = new Turret();
    public final Vision vision = new Vision(drivetrain, turret);
    public final Intake intake = new Intake();
    public final Feeder feeder = new Feeder();
    private final RobotMusic robotMusic = new RobotMusic();

    public RobotContainer() {
        configureBindings();
    }

    public void robotPeriodic() {
        robotMusic.update();
        turret.updateControlAndTelemetry();
        vision.updateControlAndTelemetry();
        intake.updateControlAndTelemetry();
        feeder.updateControlAndTelemetry();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * maxSpeedMetersPerSecond) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * maxSpeedMetersPerSecond) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * maxAngularRateRadiansPerSecond) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().onTrue(Commands.runOnce(intake::moveToStowedSetpoint, intake));
        joystick.b().onTrue(Commands.runOnce(intake::startHoming, intake));
        joystick.rightBumper().onTrue(Commands.runOnce(intake::moveToDeployedSetpoint, intake));
        joystick.rightBumper().whileTrue(Commands.startEnd(intake::runRollersIn, intake::stopRollers, intake));
        joystick.leftTrigger().whileTrue(Commands.startEnd(intake::runRollersOut, intake::stopRollers, intake));
        joystick.y()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .whileTrue(Commands.startEnd(feeder::runAll, feeder::stopAll, feeder));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.povLeft().onTrue(Commands.runOnce(turret::stepTargetHeadingLeft, turret));
        joystick.povRight().onTrue(Commands.runOnce(turret::stepTargetHeadingRight, turret));
        joystick.povUp().onTrue(Commands.runOnce(turret::stepTargetPitchUp, turret));
        joystick.povDown().onTrue(Commands.runOnce(turret::stepTargetPitchDown, turret));
        joystick.x()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .onTrue(Commands.runOnce(turret::startPitchHoming, turret));
        joystick.rightTrigger().whileTrue(Commands.startEnd(turret::runFlywheel, turret::stopFlywheel));
        joystick.leftBumper().whileTrue(Commands.startEnd(turret::runSerializer, turret::stopSerializer));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
