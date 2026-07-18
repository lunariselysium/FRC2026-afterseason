// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {
    private final CANBus feederCanBus = new CANBus(FeederConstants.kMotorCanBus);

    private final TalonFX floorMotor = new TalonFX(
        FeederConstants.kFloorMotorCanId,
        feederCanBus
    );
    private final TalonFX handoffWheelMotor = new TalonFX(
        FeederConstants.kHandoffWheelMotorCanId,
        feederCanBus
    );
    private final TalonFX beltLeaderMotor = new TalonFX(
        FeederConstants.kBeltLeaderMotorCanId,
        feederCanBus
    );
    private final TalonFX beltFollowerMotor = new TalonFX(
        FeederConstants.kBeltFollowerMotorCanId,
        feederCanBus
    );
    private final FeederJamRecoveryController jamRecoveryController =
        new FeederJamRecoveryController(
            FeederConstants.kJamQualificationSeconds,
            FeederConstants.kJamReverseSeconds
        );

    private double targetFloorOutput;
    private double targetBeltOutput;
    private double targetHandoffWheelOutput;
    private double appliedFloorOutput;
    private double appliedBeltOutput;
    private double appliedHandoffWheelOutput;
    private double floorStatorCurrentAmps;
    private double handoffWheelStatorCurrentAmps;
    private double beltLeaderStatorCurrentAmps;
    private double beltFollowerStatorCurrentAmps;
    private double floorVelocityRotationsPerSecond;
    private double handoffWheelVelocityRotationsPerSecond;
    private double beltLeaderVelocityRotationsPerSecond;
    private double beltFollowerVelocityRotationsPerSecond;
    private boolean floorJammed;
    private boolean handoffWheelJammed;
    private boolean beltLeaderJammed;
    private boolean beltFollowerJammed;

    public Feeder() {
        floorMotor.setNeutralMode(NeutralModeValue.Coast);
        handoffWheelMotor.setNeutralMode(NeutralModeValue.Coast);
        beltLeaderMotor.setNeutralMode(NeutralModeValue.Coast);
        beltFollowerMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(FeederConstants.kSupplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(FeederConstants.kStatorCurrentLimitAmps)
                .withStatorCurrentLimitEnable(true)
        );
        floorMotor.getConfigurator().apply(motorConfiguration);
        handoffWheelMotor.getConfigurator().apply(motorConfiguration);
        beltLeaderMotor.getConfigurator().apply(motorConfiguration);
        beltFollowerMotor.getConfigurator().apply(motorConfiguration);

        beltFollowerMotor.setControl(
            new Follower(
                FeederConstants.kBeltLeaderMotorCanId,
                getBeltFollowerMotorAlignment()
            )
        );
    }

    public void runAll() {
        setTargetOutputs(
            FeederConstants.kFloorMotorOutput,
            FeederConstants.kBeltMotorOutput,
            FeederConstants.kHandoffWheelMotorOutput
        );
    }

    public void runAllWithUpperFeedScale(double upperFeedOutputScale) {
        TargetOutputs outputs = calculateUpperFeedScaledOutputs(upperFeedOutputScale);
        setTargetOutputs(
            outputs.floorOutput(),
            outputs.beltOutput(),
            outputs.handoffWheelOutput()
        );
    }

    public void reverseAll() {
        TargetOutputs outputs = calculateReversedOutputs();
        setTargetOutputs(
            outputs.floorOutput(),
            outputs.beltOutput(),
            outputs.handoffWheelOutput()
        );
    }

    public void runFloor() {
        setFloorOutput(FeederConstants.kFloorMotorOutput);
    }

    public void runBelts() {
        setBeltOutput(FeederConstants.kBeltMotorOutput);
    }

    public void runHandoffWheel() {
        setHandoffWheelOutput(FeederConstants.kHandoffWheelMotorOutput);
    }

    public void stopAll() {
        setTargetOutputs(0.0, 0.0, 0.0);
    }

    public void stopFloor() {
        setFloorOutput(0.0);
    }

    public void stopBelts() {
        setBeltOutput(0.0);
    }

    public void stopHandoffWheel() {
        setHandoffWheelOutput(0.0);
    }

    public void setTargetOutputs(double floorOutput, double beltOutput, double handoffWheelOutput) {
        setFloorOutput(floorOutput);
        setBeltOutput(beltOutput);
        setHandoffWheelOutput(handoffWheelOutput);
    }

    public void setFloorOutput(double floorOutput) {
        targetFloorOutput = clampMotorOutput(floorOutput);
    }

    public void setBeltOutput(double beltOutput) {
        targetBeltOutput = clampMotorOutput(beltOutput);
    }

    public void setHandoffWheelOutput(double handoffWheelOutput) {
        targetHandoffWheelOutput = clampMotorOutput(handoffWheelOutput);
    }

    public double getTargetFloorOutput() {
        return targetFloorOutput;
    }

    public double getTargetBeltOutput() {
        return targetBeltOutput;
    }

    public double getTargetHandoffWheelOutput() {
        return targetHandoffWheelOutput;
    }

    public double getAppliedFloorOutput() {
        return appliedFloorOutput;
    }

    public double getAppliedBeltOutput() {
        return appliedBeltOutput;
    }

    public double getAppliedHandoffWheelOutput() {
        return appliedHandoffWheelOutput;
    }

    public boolean isRunning() {
        return targetFloorOutput != 0.0
            || targetBeltOutput != 0.0
            || targetHandoffWheelOutput != 0.0;
    }

    public boolean isJamRecoveryActive() {
        return jamRecoveryController.isRecoveryActive();
    }

    public void updateControlAndTelemetry(boolean publishTelemetry) {
        if (DriverStation.isDisabled()) {
            stopAll();
        }

        updateMeasurementsAndJamStatus();
        FeederJamRecoveryController.OutputMode outputMode = jamRecoveryController.update(
            Timer.getFPGATimestamp(),
            isForwardFeedRequested(),
            floorJammed,
            handoffWheelJammed,
            beltLeaderJammed,
            beltFollowerJammed
        );
        TargetOutputs appliedOutputs = outputMode == FeederJamRecoveryController.OutputMode.REVERSE
            ? calculateReversedOutputs()
            : new TargetOutputs(
                targetFloorOutput,
                targetBeltOutput,
                targetHandoffWheelOutput
            );

        setFloorMotorOutput(appliedOutputs.floorOutput());
        setBeltMotorOutput(appliedOutputs.beltOutput());
        setHandoffWheelMotorOutput(appliedOutputs.handoffWheelOutput());

        if (!publishTelemetry) {
            return;
        }

        SmartDashboard.putNumber("Feeder/FloorMotorOutput", getAppliedFloorOutput());
        SmartDashboard.putNumber("Feeder/BeltMotorOutput", getAppliedBeltOutput());
        SmartDashboard.putNumber("Feeder/HandoffWheelMotorOutput", getAppliedHandoffWheelOutput());
        SmartDashboard.putBoolean("Feeder/Running", isRunning());
        SmartDashboard.putNumber("Feeder/FloorStatorCurrentAmps", floorStatorCurrentAmps);
        SmartDashboard.putNumber(
            "Feeder/HandoffWheelStatorCurrentAmps",
            handoffWheelStatorCurrentAmps
        );
        SmartDashboard.putNumber(
            "Feeder/BeltLeaderStatorCurrentAmps",
            beltLeaderStatorCurrentAmps
        );
        SmartDashboard.putNumber(
            "Feeder/BeltFollowerStatorCurrentAmps",
            beltFollowerStatorCurrentAmps
        );
        SmartDashboard.putNumber(
            "Feeder/FloorVelocityRotationsPerSecond",
            floorVelocityRotationsPerSecond
        );
        SmartDashboard.putNumber(
            "Feeder/HandoffWheelVelocityRotationsPerSecond",
            handoffWheelVelocityRotationsPerSecond
        );
        SmartDashboard.putNumber(
            "Feeder/BeltLeaderVelocityRotationsPerSecond",
            beltLeaderVelocityRotationsPerSecond
        );
        SmartDashboard.putNumber(
            "Feeder/BeltFollowerVelocityRotationsPerSecond",
            beltFollowerVelocityRotationsPerSecond
        );
        SmartDashboard.putBoolean("Feeder/FloorJammed", floorJammed);
        SmartDashboard.putBoolean("Feeder/HandoffWheelJammed", handoffWheelJammed);
        SmartDashboard.putBoolean("Feeder/BeltLeaderJammed", beltLeaderJammed);
        SmartDashboard.putBoolean("Feeder/BeltFollowerJammed", beltFollowerJammed);
        SmartDashboard.putBoolean("Feeder/JamRecoveryActive", isJamRecoveryActive());
    }

    private MotorAlignmentValue getBeltFollowerMotorAlignment() {
        return FeederConstants.kBeltFollowerOpposesLeader
            ? MotorAlignmentValue.Opposed
            : MotorAlignmentValue.Aligned;
    }

    private double clampMotorOutput(double motorOutput) {
        return MathUtil.clamp(motorOutput, -1.0, 1.0);
    }

    static TargetOutputs calculateUpperFeedScaledOutputs(double upperFeedOutputScale) {
        double clampedOutputScale = MathUtil.clamp(upperFeedOutputScale, 0.0, 1.0);
        return new TargetOutputs(
            FeederConstants.kFloorMotorOutput,
            FeederConstants.kBeltMotorOutput * clampedOutputScale,
            FeederConstants.kHandoffWheelMotorOutput * clampedOutputScale
        );
    }

    static TargetOutputs calculateReversedOutputs() {
        return new TargetOutputs(
            -FeederConstants.kFloorMotorOutput,
            -FeederConstants.kBeltMotorOutput,
            -FeederConstants.kHandoffWheelMotorOutput
        );
    }

    static boolean isMotorJammed(
        double requestedOutput,
        double statorCurrentAmps,
        double velocityRotationsPerSecond
    ) {
        if (requestedOutput < FeederConstants.kJamMinimumCommandedOutput) {
            return false;
        }

        return Math.abs(statorCurrentAmps) >= FeederConstants.kJamCurrentThresholdAmps
            || Math.abs(velocityRotationsPerSecond)
                <= FeederConstants.kJamVelocityThresholdRotationsPerSecond;
    }

    record TargetOutputs(
        double floorOutput,
        double beltOutput,
        double handoffWheelOutput
    ) {}

    private boolean isForwardFeedRequested() {
        return targetFloorOutput >= FeederConstants.kJamMinimumCommandedOutput
            || targetBeltOutput >= FeederConstants.kJamMinimumCommandedOutput
            || targetHandoffWheelOutput >= FeederConstants.kJamMinimumCommandedOutput;
    }

    private void updateMeasurementsAndJamStatus() {
        floorStatorCurrentAmps = floorMotor.getStatorCurrent().getValueAsDouble();
        handoffWheelStatorCurrentAmps = handoffWheelMotor.getStatorCurrent().getValueAsDouble();
        beltLeaderStatorCurrentAmps = beltLeaderMotor.getStatorCurrent().getValueAsDouble();
        beltFollowerStatorCurrentAmps = beltFollowerMotor.getStatorCurrent().getValueAsDouble();
        floorVelocityRotationsPerSecond = floorMotor.getVelocity().getValueAsDouble();
        handoffWheelVelocityRotationsPerSecond = handoffWheelMotor
            .getVelocity()
            .getValueAsDouble();
        beltLeaderVelocityRotationsPerSecond = beltLeaderMotor
            .getVelocity()
            .getValueAsDouble();
        beltFollowerVelocityRotationsPerSecond = beltFollowerMotor
            .getVelocity()
            .getValueAsDouble();

        floorJammed = isMotorJammed(
            targetFloorOutput,
            floorStatorCurrentAmps,
            floorVelocityRotationsPerSecond
        );
        handoffWheelJammed = isMotorJammed(
            targetHandoffWheelOutput,
            handoffWheelStatorCurrentAmps,
            handoffWheelVelocityRotationsPerSecond
        );
        beltLeaderJammed = isMotorJammed(
            targetBeltOutput,
            beltLeaderStatorCurrentAmps,
            beltLeaderVelocityRotationsPerSecond
        );
        beltFollowerJammed = isMotorJammed(
            targetBeltOutput,
            beltFollowerStatorCurrentAmps,
            beltFollowerVelocityRotationsPerSecond
        );
    }

    private void setFloorMotorOutput(double floorOutput) {
        appliedFloorOutput = FeederConstants.kFloorMotorOutputSign * floorOutput;
        floorMotor.set(appliedFloorOutput);
    }

    private void setBeltMotorOutput(double beltOutput) {
        appliedBeltOutput = FeederConstants.kBeltMotorOutputSign * beltOutput;
        beltLeaderMotor.set(appliedBeltOutput);
    }

    private void setHandoffWheelMotorOutput(double handoffWheelOutput) {
        appliedHandoffWheelOutput = FeederConstants.kHandoffWheelMotorOutputSign * handoffWheelOutput;
        handoffWheelMotor.set(appliedHandoffWheelOutput);
    }
}
