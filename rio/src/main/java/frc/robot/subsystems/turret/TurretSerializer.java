// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TurretSerializerConstants;

public class TurretSerializer {
    private final TalonFX serializerMotor = new TalonFX(
        TurretSerializerConstants.kMotorCanId,
        new CANBus(TurretSerializerConstants.kMotorCanBus)
    );

    private double targetMotorOutput;
    private double appliedMotorOutput;

    public TurretSerializer() {
        serializerMotor.getConfigurator().apply(
            new TalonFXConfiguration().withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(TurretSerializerConstants.kSupplyCurrentLimitAmps)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(TurretSerializerConstants.kStatorCurrentLimitAmps)
                    .withStatorCurrentLimitEnable(true)
            )
        );
        serializerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void run() {
        runAtScale(1.0);
    }

    public void runAtScale(double outputScale) {
        targetMotorOutput = TurretSerializerConstants.kMotorOutput
            * MathUtil.clamp(outputScale, 0.0, 1.0);
    }

    public void reverse() {
        targetMotorOutput = calculateReverseOutput();
    }

    public void stop() {
        targetMotorOutput = 0.0;
    }

    public void updateControl() {
        if (DriverStation.isDisabled()) {
            stop();
            setMotorOutput(0.0);
            return;
        }

        if (!isRunning()) {
            setMotorOutput(0.0);
            return;
        }

        setMotorOutput(targetMotorOutput);
    }

    public boolean isRunning() {
        return targetMotorOutput != 0.0;
    }

    public double getAppliedMotorOutput() {
        return appliedMotorOutput;
    }

    static double calculateReverseOutput() {
        return -TurretSerializerConstants.kMotorOutput;
    }

    private void setMotorOutput(double motorOutput) {
        appliedMotorOutput = TurretSerializerConstants.kMotorOutputSign * motorOutput;
        serializerMotor.set(appliedMotorOutput);
    }
}
