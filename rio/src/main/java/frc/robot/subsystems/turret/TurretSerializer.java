// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TurretSerializerConstants;

public class TurretSerializer {
    private final TalonFX serializerMotor = new TalonFX(
        TurretSerializerConstants.kMotorCanId,
        new CANBus(TurretSerializerConstants.kMotorCanBus)
    );

    private boolean running;
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
        running = true;
    }

    public void stop() {
        running = false;
    }

    public void updateControl() {
        if (DriverStation.isDisabled()) {
            running = false;
            setMotorOutput(0.0);
            return;
        }

        if (!running) {
            setMotorOutput(0.0);
            return;
        }

        setMotorOutput(TurretSerializerConstants.kMotorOutput);
    }

    public boolean isRunning() {
        return running;
    }

    public double getAppliedMotorOutput() {
        return appliedMotorOutput;
    }

    private void setMotorOutput(double motorOutput) {
        appliedMotorOutput = TurretSerializerConstants.kMotorOutputSign * motorOutput;
        serializerMotor.set(appliedMotorOutput);
    }
}
