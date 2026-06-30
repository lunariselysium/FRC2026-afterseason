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

    private double targetFloorOutput;
    private double targetBeltOutput;
    private double targetHandoffWheelOutput;
    private double appliedFloorOutput;
    private double appliedBeltOutput;
    private double appliedHandoffWheelOutput;

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

    public void updateControlAndTelemetry() {
        if (DriverStation.isDisabled()) {
            stopAll();
        }

        setFloorMotorOutput(targetFloorOutput);
        setBeltMotorOutput(targetBeltOutput);
        setHandoffWheelMotorOutput(targetHandoffWheelOutput);

        SmartDashboard.putNumber("Feeder/FloorMotorOutput", getAppliedFloorOutput());
        SmartDashboard.putNumber("Feeder/BeltMotorOutput", getAppliedBeltOutput());
        SmartDashboard.putNumber("Feeder/HandoffWheelMotorOutput", getAppliedHandoffWheelOutput());
        SmartDashboard.putBoolean("Feeder/Running", isRunning());
    }

    private MotorAlignmentValue getBeltFollowerMotorAlignment() {
        return FeederConstants.kBeltFollowerOpposesLeader
            ? MotorAlignmentValue.Opposed
            : MotorAlignmentValue.Aligned;
    }

    private double clampMotorOutput(double motorOutput) {
        return MathUtil.clamp(motorOutput, -1.0, 1.0);
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
