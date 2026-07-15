// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.TurretFlywheelConstants;
import frc.robot.Constants.TurretPitchConstants;
import frc.robot.subsystems.turret.TurretFlywheel;
import frc.robot.subsystems.turret.TurretHeading;
import frc.robot.subsystems.turret.TurretPitch;
import frc.robot.subsystems.turret.TurretSerializer;

public class Turret extends SubsystemBase {
    private final TurretHeading heading = new TurretHeading();
    private final TurretPitch pitch = new TurretPitch();
    private final TurretFlywheel flywheel = new TurretFlywheel();
    private final TurretSerializer serializer = new TurretSerializer();

    private final SysIdRoutine headingSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(TurretConstants.kSysIdQuasistaticRampRateVoltsPerSecond).per(Second),
            Volts.of(TurretConstants.kSysIdDynamicStepVolts),
            Seconds.of(TurretConstants.kSysIdTimeoutSeconds)
        ),
        new SysIdRoutine.Mechanism(
            voltage -> heading.runSysIdVoltage(voltage.in(Volts)),
            log -> log.motor("turret-heading")
                .voltage(Volts.of(heading.getAppliedSysIdMechanismVoltage()))
                .angularPosition(Rotations.of(heading.getHeadingDegrees() / 360.0))
                .angularVelocity(RotationsPerSecond.of(heading.getHeadingVelocityDegreesPerSecond() / 360.0))
                .current(Amps.of(heading.getStatorCurrentAmps())),
            this,
            "TurretHeading"
        )
    );

    private final SysIdRoutine pitchSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(TurretPitchConstants.kSysIdQuasistaticRampRateVoltsPerSecond).per(Second),
            Volts.of(TurretPitchConstants.kSysIdDynamicStepVolts),
            Seconds.of(TurretPitchConstants.kSysIdTimeoutSeconds)
        ),
        new SysIdRoutine.Mechanism(
            voltage -> pitch.runSysIdVoltage(voltage.in(Volts)),
            log -> log.motor("turret-pitch")
                .voltage(Volts.of(pitch.getAppliedSysIdMechanismVoltage()))
                .angularPosition(Rotations.of(pitch.getPitchDegrees() / 360.0))
                .angularVelocity(RotationsPerSecond.of(pitch.getPitchVelocityDegreesPerSecond() / 360.0))
                .current(Amps.of(pitch.getStatorCurrentAmps())),
            this,
            "TurretPitch"
        )
    );

    private final SysIdRoutine flywheelSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(TurretFlywheelConstants.kSysIdQuasistaticRampRateVoltsPerSecond).per(Second),
            Volts.of(TurretFlywheelConstants.kSysIdDynamicStepVolts),
            Seconds.of(TurretFlywheelConstants.kSysIdTimeoutSeconds)
        ),
        new SysIdRoutine.Mechanism(
            voltage -> flywheel.runSysIdVoltage(voltage.in(Volts)),
            log -> log.motor("turret-flywheel-leader")
                .voltage(Volts.of(flywheel.getAppliedSysIdVoltage()))
                .angularPosition(Rotations.of(flywheel.getPositionRotations()))
                .angularVelocity(RotationsPerSecond.of(flywheel.getVelocityRotationsPerSecond()))
                .current(Amps.of(flywheel.getLeaderStatorCurrentAmps())),
            this,
            "TurretFlywheel"
        )
    );

    public void stepTargetHeadingLeft() {
        heading.stepTargetLeft();
    }

    public void stepTargetHeadingRight() {
        heading.stepTargetRight();
    }

    public void resetHeadingEncoderRotationToTargetHeading() {
        heading.resetEncoderRotationToTargetHeading();
    }

    public void shiftHeadingEncoderRotationLeft() {
        heading.shiftEncoderRotationTowardPositiveHeading();
    }

    public void shiftHeadingEncoderRotationRight() {
        heading.shiftEncoderRotationTowardNegativeHeading();
    }

    public double getHeadingDegrees() {
        return heading.getHeadingDegrees();
    }

    public double getTargetHeadingDegrees() {
        return heading.getTargetHeadingDegrees();
    }

    public void setTargetHeadingDegrees(double headingDegrees) {
        heading.setTargetHeadingDegrees(headingDegrees);
    }

    public boolean isHeadingAtTarget() {
        return heading.isAtTarget();
    }

    public void stepTargetPitchUp() {
        pitch.stepTargetUp();
    }

    public void stepTargetPitchDown() {
        pitch.stepTargetDown();
    }

    public void startPitchHoming() {
        pitch.startHoming();
    }

    public double getPitchDegrees() {
        return pitch.getPitchDegrees();
    }

    public double getTargetPitchDegrees() {
        return pitch.getTargetPitchDegrees();
    }

    public void setTargetPitchDegrees(double pitchDegrees) {
        pitch.setTargetPitchDegrees(pitchDegrees);
    }

    public boolean isPitchHomed() {
        return pitch.isHomed();
    }

    public boolean isPitchAtTarget() {
        return pitch.isAtTarget();
    }

    public void runFlywheel() {
        flywheel.run();
    }

    public void runFlywheelAtVelocityRotationsPerSecond(double velocityRotationsPerSecond) {
        flywheel.run(velocityRotationsPerSecond);
    }

    public Command runFlywheelAtVelocityCommand(DoubleSupplier velocityRotationsPerSecondSupplier) {
        return runEnd(
            () -> runFlywheelAtVelocityRotationsPerSecond(
                velocityRotationsPerSecondSupplier.getAsDouble()
            ),
            this::stopFlywheel
        );
    }

    public void stopFlywheel() {
        flywheel.stop();
    }

    public boolean isFlywheelAtTarget() {
        return flywheel.isAtTarget();
    }

    public boolean isFlywheelReadyToShoot() {
        return flywheel.isReadyToShoot();
    }

    public boolean isAnySysIdActive() {
        return heading.isSysIdActive()
            || pitch.isSysIdActive()
            || flywheel.isSysIdActive();
    }

    public double getFlywheelVelocityRotationsPerSecond() {
        return flywheel.getVelocityRotationsPerSecond();
    }

    public double getTargetFlywheelVelocityRotationsPerSecond() {
        return flywheel.getTargetVelocityRotationsPerSecond();
    }

    public double getFlywheelFeedingLoadFeedforwardVolts() {
        return flywheel.getAppliedFeedingLoadFeedforwardVolts();
    }

    public void runSerializer() {
        serializer.run();
    }

    public void runSerializerAtScale(double outputScale) {
        serializer.runAtScale(outputScale);
    }

    public void reverseSerializer() {
        serializer.reverse();
    }

    public void stopSerializer() {
        serializer.stop();
    }

    public void stopShotOutputs() {
        stopFlywheel();
        stopSerializer();
    }

    public Command sysIdHeadingQuasistatic(Direction direction) {
        return runOnce(this::prepareHeadingSysId).andThen(headingSysIdRoutine.quasistatic(direction));
    }

    public Command sysIdHeadingDynamic(Direction direction) {
        return runOnce(this::prepareHeadingSysId).andThen(headingSysIdRoutine.dynamic(direction));
    }

    public Command sysIdPitchQuasistatic(Direction direction) {
        return runOnce(this::preparePitchSysId).andThen(pitchSysIdRoutine.quasistatic(direction));
    }

    public Command sysIdPitchDynamic(Direction direction) {
        return runOnce(this::preparePitchSysId).andThen(pitchSysIdRoutine.dynamic(direction));
    }

    public Command sysIdFlywheelQuasistatic(Direction direction) {
        return runOnce(this::prepareFlywheelSysId).andThen(flywheelSysIdRoutine.quasistatic(direction));
    }

    public Command sysIdFlywheelDynamic(Direction direction) {
        return runOnce(this::prepareFlywheelSysId).andThen(flywheelSysIdRoutine.dynamic(direction));
    }

    public void updateControlAndTelemetry() {
        heading.updateControl();
        pitch.updateControl();
        flywheel.updateControl(serializer.isRunning());
        serializer.updateControl();

        SmartDashboard.putString("Turret/HeadingStatus", heading.getStatus());
        SmartDashboard.putNumber("Turret/HeadingDegrees", heading.getHeadingDegrees());
        SmartDashboard.putNumber("Turret/HeadingTargetDegrees", heading.getTargetHeadingDegrees());
        SmartDashboard.putNumber("Turret/HeadingMotorOutput", heading.getAppliedMotorOutput());
        SmartDashboard.putNumber("Turret/HeadingVelocityDegreesPerSecond", heading.getHeadingVelocityDegreesPerSecond());
        SmartDashboard.putNumber("Turret/HeadingSysIdVoltage", heading.getAppliedSysIdMechanismVoltage());
        SmartDashboard.putNumber("Turret/HeadingMeasuredMechanismVoltage", heading.getMeasuredMechanismVoltage());
        SmartDashboard.putNumber("Turret/HeadingStatorCurrentAmps", heading.getStatorCurrentAmps());
        SmartDashboard.putBoolean("Turret/HeadingSysIdActive", heading.isSysIdActive());

        SmartDashboard.putNumber("Turret/PitchDegrees", pitch.getPitchDegrees());
        SmartDashboard.putNumber("Turret/PitchTargetDegrees", pitch.getTargetPitchDegrees());
        SmartDashboard.putNumber("Turret/PitchMotorOutput", pitch.getAppliedMotorOutput());
        SmartDashboard.putNumber("Turret/PitchVelocityDegreesPerSecond", pitch.getPitchVelocityDegreesPerSecond());
        SmartDashboard.putNumber("Turret/PitchSysIdVoltage", pitch.getAppliedSysIdMechanismVoltage());
        SmartDashboard.putNumber("Turret/PitchMeasuredMechanismVoltage", pitch.getMeasuredMechanismVoltage());
        SmartDashboard.putNumber("Turret/PitchStatorCurrentAmps", pitch.getStatorCurrentAmps());
        SmartDashboard.putBoolean("Turret/PitchHoming", pitch.isHoming());
        SmartDashboard.putBoolean("Turret/PitchHomed", pitch.isHomed());
        SmartDashboard.putBoolean("Turret/PitchHomingTimedOut", pitch.didHomingTimeOut());
        SmartDashboard.putBoolean("Turret/PitchSysIdActive", pitch.isSysIdActive());

        SmartDashboard.putNumber("Turret/FlywheelVelocityRps", flywheel.getVelocityRotationsPerSecond());
        SmartDashboard.putNumber("Turret/FlywheelTargetVelocityRps", flywheel.getTargetVelocityRotationsPerSecond());
        SmartDashboard.putNumber("Turret/FlywheelPositionRotations", flywheel.getPositionRotations());
        SmartDashboard.putNumber("Turret/FlywheelSysIdVoltage", flywheel.getAppliedSysIdVoltage());
        SmartDashboard.putNumber(
            "Turret/FlywheelFeedingLoadFeedforwardVolts",
            flywheel.getAppliedFeedingLoadFeedforwardVolts()
        );
        SmartDashboard.putNumber("Turret/FlywheelMeasuredVoltage", flywheel.getLeaderMotorVoltage());
        SmartDashboard.putNumber("Turret/FlywheelLeaderStatorCurrentAmps", flywheel.getLeaderStatorCurrentAmps());
        SmartDashboard.putBoolean("Turret/FlywheelAtTarget", flywheel.isAtTarget());
        SmartDashboard.putBoolean("Turret/FlywheelReadyToShoot", flywheel.isReadyToShoot());
        SmartDashboard.putBoolean("Turret/FlywheelRunning", flywheel.isRunning());
        SmartDashboard.putBoolean("Turret/FlywheelSysIdActive", flywheel.isSysIdActive());

        SmartDashboard.putBoolean("Turret/SerializerRunning", serializer.isRunning());
    }

    private void prepareHeadingSysId() {
        pitch.stopSysId();
        flywheel.stopSysId();
        flywheel.stop();
        serializer.stop();
        heading.prepareSysId();
    }

    private void preparePitchSysId() {
        heading.stopSysId();
        flywheel.stopSysId();
        flywheel.stop();
        serializer.stop();
        pitch.prepareSysId();
    }

    private void prepareFlywheelSysId() {
        heading.stopSysId();
        pitch.stopSysId();
        serializer.stop();
        flywheel.prepareSysId();
    }
}
