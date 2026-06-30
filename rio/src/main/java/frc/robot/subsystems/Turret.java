// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretFlywheel;
import frc.robot.subsystems.turret.TurretHeading;
import frc.robot.subsystems.turret.TurretPitch;
import frc.robot.subsystems.turret.TurretSerializer;

public class Turret extends SubsystemBase {
    private final TurretHeading heading = new TurretHeading();
    private final TurretPitch pitch = new TurretPitch();
    private final TurretFlywheel flywheel = new TurretFlywheel();
    private final TurretSerializer serializer = new TurretSerializer();

    public void stepTargetHeadingLeft() {
        heading.stepTargetLeft();
    }

    public void stepTargetHeadingRight() {
        heading.stepTargetRight();
    }

    public double getHeadingDegrees() {
        return heading.getHeadingDegrees();
    }

    public void setTargetHeadingDegrees(double headingDegrees) {
        heading.setTargetHeadingDegrees(headingDegrees);
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

    public void runFlywheel() {
        flywheel.run();
    }

    public void stopFlywheel() {
        flywheel.stop();
    }

    public void runSerializer() {
        serializer.run();
    }

    public void stopSerializer() {
        serializer.stop();
    }

    public void updateControlAndTelemetry() {
        heading.updateControl();
        pitch.updateControl();
        flywheel.updateControl();
        serializer.updateControl();

        SmartDashboard.putString("Turret/HeadingStatus", heading.getStatus());
        SmartDashboard.putNumber("Turret/HeadingDegrees", heading.getHeadingDegrees());
        SmartDashboard.putNumber("Turret/HeadingTargetDegrees", heading.getTargetHeadingDegrees());
        SmartDashboard.putNumber("Turret/HeadingMotorOutput", heading.getAppliedMotorOutput());

        SmartDashboard.putNumber("Turret/PitchDegrees", pitch.getPitchDegrees());
        SmartDashboard.putNumber("Turret/PitchTargetDegrees", pitch.getTargetPitchDegrees());
        SmartDashboard.putNumber("Turret/PitchMotorOutput", pitch.getAppliedMotorOutput());
        SmartDashboard.putBoolean("Turret/PitchHoming", pitch.isHoming());
        SmartDashboard.putBoolean("Turret/PitchHomed", pitch.isHomed());

        SmartDashboard.putNumber("Turret/FlywheelVelocityRps", flywheel.getVelocityRotationsPerSecond());
        SmartDashboard.putBoolean("Turret/FlywheelAtTarget", flywheel.isAtTarget());
        SmartDashboard.putBoolean("Turret/FlywheelRunning", flywheel.isRunning());

        SmartDashboard.putBoolean("Turret/SerializerRunning", serializer.isRunning());
    }
}
