// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.TurretFlywheelConstants;

public class TurretFlywheel {
    private final CANBus flywheelCanBus = new CANBus(TurretFlywheelConstants.kMotorCanBus);

    private final TalonFX leaderMotor = new TalonFX(
        TurretFlywheelConstants.kLeaderMotorCanId,
        flywheelCanBus
    );
    private final TalonFX followerMotor = new TalonFX(
        TurretFlywheelConstants.kFollowerMotorCanId,
        flywheelCanBus
    );

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    private boolean running;

    public TurretFlywheel() {
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(TurretFlywheelConstants.kSupplyCurrentLimitAmps)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(TurretFlywheelConstants.kStatorCurrentLimitAmps)
            .withStatorCurrentLimitEnable(true);

        TalonFXConfiguration leaderConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withSlot0(
                new Slot0Configs()
                    .withKP(TurretFlywheelConstants.kFlywheelKp)
                    .withKV(TurretFlywheelConstants.kFlywheelKv)
                    .withKS(TurretFlywheelConstants.kFlywheelKs)
            );

        leaderMotor.getConfigurator().apply(leaderConfiguration);
        followerMotor.getConfigurator().apply(
            new TalonFXConfiguration().withCurrentLimits(currentLimits)
        );
        leaderMotor.setNeutralMode(NeutralModeValue.Coast);
        followerMotor.setNeutralMode(NeutralModeValue.Coast);
        followerMotor.setControl(
            new Follower(
                TurretFlywheelConstants.kLeaderMotorCanId,
                getFollowerMotorAlignment()
            )
        );
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
            leaderMotor.stopMotor();
            return;
        }

        if (!running) {
            leaderMotor.stopMotor();
            return;
        }

        leaderMotor.setControl(
            velocityRequest.withVelocity(TurretFlywheelConstants.kTargetVelocityRotationsPerSecond)
        );
    }

    public double getVelocityRotationsPerSecond() {
        return leaderMotor.getVelocity().getValueAsDouble();
    }

    public double getTargetVelocityRotationsPerSecond() {
        return running ? TurretFlywheelConstants.kTargetVelocityRotationsPerSecond : 0.0;
    }

    public boolean isRunning() {
        return running;
    }

    public boolean isAtTarget() {
        return running
            && Math.abs(getVelocityRotationsPerSecond() - TurretFlywheelConstants.kTargetVelocityRotationsPerSecond)
                <= TurretFlywheelConstants.kVelocityToleranceRotationsPerSecond;
    }

    private MotorAlignmentValue getFollowerMotorAlignment() {
        return TurretFlywheelConstants.kFollowerOpposesLeader
            ? MotorAlignmentValue.Opposed
            : MotorAlignmentValue.Aligned;
    }
}
