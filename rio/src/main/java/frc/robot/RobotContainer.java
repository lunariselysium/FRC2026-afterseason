// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.TurretPitchConstants;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ShotAimCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.scoring.AutoScoreIntakeAssist;
import frc.robot.scoring.AutoScoreIntakeAssist.IntakeRequest;
import frc.robot.scoring.ScoringTelemetry;
import frc.robot.scoring.ShotTuningControls;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private static final Voltage kDriveSpeedTestVoltage = Volts.of(12.0);

    private final double maxSpeedMetersPerSecond =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.2;
    private final double maxAngularRateRadiansPerSecond =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.6;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeedMetersPerSecond * 0.1)
            .withRotationalDeadband(maxAngularRateRadiansPerSecond * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(maxSpeedMetersPerSecond);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Trigger backButton = joystick.back();
    private final Trigger startButton = joystick.start();
    private final Trigger leftTrigger = joystick.leftTrigger();
    private final Trigger rightTrigger = joystick.rightTrigger();
    private final Trigger rightStickButton = joystick.rightStick();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Turret turret = new Turret();
    public final Vision vision = new Vision(drivetrain, turret);
    public final Intake intake = new Intake();
    public final Feeder feeder = new Feeder();
    private final ScoreCommand scoreCommand = new ScoreCommand(drivetrain, turret, feeder, vision);
    private final AutoScoreIntakeAssist autoScoreIntakeAssist = new AutoScoreIntakeAssist();
    private boolean pathPlannerIntakeRollersRequested;
    private boolean pathPlannerAutoScoreRequested;
    private final Trigger pathPlannerIntakeRollersActive = new Trigger(
        () -> DriverStation.isAutonomousEnabled() && pathPlannerIntakeRollersRequested
    );
    private final Trigger pathPlannerAutoScoreActive = new Trigger(
        () -> DriverStation.isAutonomousEnabled() && pathPlannerAutoScoreRequested
    );
    private final ScoringTelemetry scoringTelemetry =
        new ScoringTelemetry(drivetrain, turret, vision);
    private final ShotTuningControls shotTuningControls = new ShotTuningControls(turret);
    private final RobotMusic robotMusic = new RobotMusic();

    public RobotContainer() {
        configurePathPlannerBindings();
        configureBindings();
    }

    public void robotPeriodic() {
        robotMusic.update();
        shotTuningControls.update(isNormalScoringRequested(), leftTrigger.getAsBoolean());
        turret.updateControlAndTelemetry();
        vision.updateControlAndTelemetry();
        scoringTelemetry.update();
        updateAutoScoreIntakeAssist();
        intake.updateControlAndTelemetry();
        feeder.updateControlAndTelemetry();
    }

    private void configurePathPlannerBindings() {
        NamedCommands.registerCommand(
            "Intake Deploy",
            Commands.runOnce(intake::moveToDeployedSetpoint, intake)
        );
        NamedCommands.registerCommand(
            "Intake Retract",
            Commands.runOnce(intake::moveToStowedSetpoint, intake)
        );
        NamedCommands.registerCommand(
            "Intake Roller Start",
            Commands.runOnce(() -> pathPlannerIntakeRollersRequested = true)
        );
        NamedCommands.registerCommand(
            "Intake Roller Stop",
            Commands.runOnce(() -> pathPlannerIntakeRollersRequested = false)
        );
        NamedCommands.registerCommand(
            "Auto Score Start",
            Commands.runOnce(() -> pathPlannerAutoScoreRequested = true)
        );
        NamedCommands.registerCommand(
            "Auto Score End",
            Commands.runOnce(() -> pathPlannerAutoScoreRequested = false)
        );
        NamedCommands.registerCommand(
            "Home Intake And Shooter Pitch",
            Commands.parallel(
                Commands.runOnce(intake::startHoming, intake),
                Commands.runOnce(turret::startPitchHoming, turret)
            )
        );

        pathPlannerIntakeRollersActive
            .onTrue(Commands.runOnce(intake::runRollersIn, intake))
            .onFalse(
                Commands.runOnce(
                    () -> {
                        pathPlannerIntakeRollersRequested = false;
                        intake.stopRollers();
                    },
                    intake
                ).ignoringDisable(true)
            );
        pathPlannerAutoScoreActive
            .whileTrue(scoreCommand)
            .onFalse(
                Commands.runOnce(
                    () -> pathPlannerAutoScoreRequested = false
                ).ignoringDisable(true)
            );
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
        turret.setDefaultCommand(
            Commands.run(
                () -> turret.setTargetPitchDegrees(TurretPitchConstants.kDefaultPitchDegrees),
                turret
            )
        );

        joystick.a()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .onTrue(Commands.runOnce(intake::moveToStowedSetpoint, intake));
        joystick.b()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .onTrue(Commands.runOnce(intake::startHoming, intake));
        joystick.leftBumper()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .whileTrue(
                Commands.startEnd(
                    () -> {
                        intake.moveToDeployedSetpoint();
                        intake.runRollersIn();
                    },
                    intake::stopRollers,
                    intake
                )
            );
        rightTrigger.whileTrue(
            turret.runFlywheelAtVelocityCommand(
                shotTuningControls::getRequestedFlywheelRotationsPerSecond
            )
        );
        joystick.y()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .whileTrue(Commands.startEnd(feeder::runAll, feeder::stopAll, feeder));

        // Test mode only: hold Back + Y to apply 12 V straight forward to the drive motors.
        // Releasing either button immediately commands 0 V before normal drive control resumes.
        new Trigger(DriverStation::isTestEnabled)
            .and(backButton)
            .and(joystick.y())
            .whileTrue(drivetrain.driveForwardAtVoltageCommand(kDriveSpeedTestVoltage));

        // var drivetrainTranslationSysId = joystick.leftBumper().negate().and(joystick.rightBumper().negate());
        // var drivetrainSteerSysId = joystick.leftBumper().and(joystick.rightBumper().negate());
        // var drivetrainRotationSysId = joystick.rightBumper().and(joystick.leftBumper().negate());

        // // Drivetrain SysId: Back = dynamic, Start = quasistatic; Y = forward, X = reverse.
        // // No bumper selects translation, left bumper selects steer, right bumper selects rotation.
        // joystick.back().and(joystick.y()).and(drivetrainTranslationSysId)
        //     .whileTrue(drivetrain.sysIdTranslationDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).and(drivetrainTranslationSysId)
        //     .whileTrue(drivetrain.sysIdTranslationDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).and(drivetrainTranslationSysId)
        //     .whileTrue(drivetrain.sysIdTranslationQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).and(drivetrainTranslationSysId)
        //     .whileTrue(drivetrain.sysIdTranslationQuasistatic(Direction.kReverse));

        // joystick.back().and(joystick.y()).and(drivetrainSteerSysId)
        //     .whileTrue(drivetrain.sysIdSteerDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).and(drivetrainSteerSysId)
        //     .whileTrue(drivetrain.sysIdSteerDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).and(drivetrainSteerSysId)
        //     .whileTrue(drivetrain.sysIdSteerQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).and(drivetrainSteerSysId)
        //     .whileTrue(drivetrain.sysIdSteerQuasistatic(Direction.kReverse));

        // joystick.back().and(joystick.y()).and(drivetrainRotationSysId)
        //     .whileTrue(drivetrain.sysIdRotationDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).and(drivetrainRotationSysId)
        //     .whileTrue(drivetrain.sysIdRotationDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).and(drivetrainRotationSysId)
        //     .whileTrue(drivetrain.sysIdRotationQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).and(drivetrainRotationSysId)
        //     .whileTrue(drivetrain.sysIdRotationQuasistatic(Direction.kReverse));

        // // Turret SysId chords use Back for dynamic tests and Start for quasistatic tests.
        // joystick.back().and(joystick.povLeft()).whileTrue(turret.sysIdHeadingDynamic(Direction.kForward));
        // joystick.back().and(joystick.povRight()).whileTrue(turret.sysIdHeadingDynamic(Direction.kReverse));
        // joystick.start().and(joystick.povLeft()).whileTrue(turret.sysIdHeadingQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.povRight()).whileTrue(turret.sysIdHeadingQuasistatic(Direction.kReverse));

        // joystick.back().and(joystick.povUp()).whileTrue(turret.sysIdPitchDynamic(Direction.kForward));
        // joystick.back().and(joystick.povDown()).whileTrue(turret.sysIdPitchDynamic(Direction.kReverse));
        // joystick.start().and(joystick.povUp()).whileTrue(turret.sysIdPitchQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.povDown()).whileTrue(turret.sysIdPitchQuasistatic(Direction.kReverse));

        // joystick.back().and(joystick.b()).whileTrue(turret.sysIdFlywheelDynamic(Direction.kForward));
        // joystick.back().and(joystick.a()).whileTrue(turret.sysIdFlywheelDynamic(Direction.kReverse));
        // joystick.start().and(joystick.b()).whileTrue(turret.sysIdFlywheelQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.a()).whileTrue(turret.sysIdFlywheelQuasistatic(Direction.kReverse));

        joystick.povLeft()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .onTrue(Commands.runOnce(this::stepTurretHeadingLeftFromPov));
        joystick.povRight()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .onTrue(Commands.runOnce(this::stepTurretHeadingRightFromPov));
        joystick.povUp()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .onTrue(Commands.runOnce(turret::stepTargetPitchUp));
        joystick.povDown()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .onTrue(Commands.runOnce(turret::stepTargetPitchDown));
        joystick.x()
            .and(joystick.back().negate())
            .and(joystick.start().negate())
            .onTrue(Commands.runOnce(turret::startPitchHoming));
        leftTrigger
            .and(backButton.negate())
            .and(startButton.negate())
            .whileTrue(scoreCommand);
        startButton
            .and(backButton.negate())
            .and(leftTrigger)
            .whileTrue(new ShotAimCommand(drivetrain, turret, vision));
        // joystick.leftBumper()
        //     .and(joystick.back().negate())
        //     .and(joystick.start().negate())
        //     .whileTrue(Commands.startEnd(turret::runSerializer, turret::stopSerializer));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private boolean isNormalScoringRequested() {
        return rightTrigger.getAsBoolean()
            && !backButton.getAsBoolean()
            && !startButton.getAsBoolean();
    }

    private void stepTurretHeadingLeftFromPov() {
        resetTurretHeadingEncoderRotationIfRequested();
        turret.stepTargetHeadingLeft();
    }

    private void stepTurretHeadingRightFromPov() {
        resetTurretHeadingEncoderRotationIfRequested();
        turret.stepTargetHeadingRight();
    }

    private void resetTurretHeadingEncoderRotationIfRequested() {
        if (rightStickButton.getAsBoolean()) {
            turret.resetHeadingEncoderRotationToTargetHeading();
        }
    }

    private boolean isIntakeButtonRequested() {
        return joystick.leftBumper().getAsBoolean()
            && !backButton.getAsBoolean()
            && !startButton.getAsBoolean();
    }

    private void updateAutoScoreIntakeAssist() {
        IntakeRequest intakeRequest = autoScoreIntakeAssist.update(
            scoreCommand.isActive(),
            scoreCommand.isFeeding(),
            isIntakeButtonRequested()
        );

        switch (intakeRequest) {
            case DEPLOYED -> intake.moveToDeployedSetpoint();
            case AUTO_SCORE_RETRACTED -> intake.moveToAutoScoreRetractionSetpoint();
            case IDLE -> {
            }
        }
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
