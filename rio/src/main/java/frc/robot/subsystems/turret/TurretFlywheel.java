// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
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

    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0.0);

    private double targetVelocityRotationsPerSecond =
        TurretFlywheelConstants.kTargetVelocityRotationsPerSecond;
    private double appliedSysIdVoltage;
    private boolean running;
    private boolean sysIdActive;

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
                    .withKA(TurretFlywheelConstants.kFlywheelKa)
                    .withKS(TurretFlywheelConstants.kFlywheelKs)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(
                        TurretFlywheelConstants.kMotionMagicAccelerationRotationsPerSecondSquared
                    )
                    .withMotionMagicJerk(
                        TurretFlywheelConstants.kMotionMagicJerkRotationsPerSecondCubed
                    )
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
        run(TurretFlywheelConstants.kTargetVelocityRotationsPerSecond);
    }

    public void run(double targetVelocityRotationsPerSecond) {
        this.targetVelocityRotationsPerSecond = MathUtil.clamp(
            targetVelocityRotationsPerSecond,
            0.0,
            Double.POSITIVE_INFINITY
        );
        running = true;
        sysIdActive = false;
    }

    public void stop() {
        running = false;
    }

    public void prepareSysId() {
        running = false;
        sysIdActive = false;
        setSysIdVoltage(0.0);
    }

    public void runSysIdVoltage(double voltage) {
        if (Math.abs(voltage) < 1.0e-6) {
            stopSysId();
            return;
        }

        running = false;
        sysIdActive = true;
        setSysIdVoltage(voltage);
    }

    public void stopSysId() {
        sysIdActive = false;
        setSysIdVoltage(0.0);
    }

    public void updateControl() {
        if (DriverStation.isDisabled()) {
            running = false;
            sysIdActive = false;
            stopLeaderMotor();
            return;
        }

        if (sysIdActive) {
            return;
        }

        if (!running) {
            stopLeaderMotor();
            return;
        }

        appliedSysIdVoltage = 0.0;
        leaderMotor.setControl(
            velocityRequest.withVelocity(targetVelocityRotationsPerSecond)
        );
    }

    public double getVelocityRotationsPerSecond() {
        return leaderMotor.getVelocity().getValueAsDouble();
    }

    public double getPositionRotations() {
        return leaderMotor.getPosition().getValueAsDouble();
    }

    public double getTargetVelocityRotationsPerSecond() {
        return running ? targetVelocityRotationsPerSecond : 0.0;
    }

    public double getAppliedSysIdVoltage() {
        return appliedSysIdVoltage;
    }

    public double getLeaderMotorVoltage() {
        return leaderMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getLeaderStatorCurrentAmps() {
        return leaderMotor.getStatorCurrent().getValueAsDouble();
    }

    public boolean isRunning() {
        return running;
    }

    public boolean isSysIdActive() {
        return sysIdActive;
    }

    public boolean isAtTarget() {
        return running
            && Math.abs(getVelocityRotationsPerSecond() - targetVelocityRotationsPerSecond)
                <= TurretFlywheelConstants.kVelocityToleranceRotationsPerSecond;
    }

    private MotorAlignmentValue getFollowerMotorAlignment() {
        return TurretFlywheelConstants.kFollowerOpposesLeader
            ? MotorAlignmentValue.Opposed
            : MotorAlignmentValue.Aligned;
    }

    private void setSysIdVoltage(double voltage) {
        appliedSysIdVoltage = voltage;
        leaderMotor.setVoltage(voltage);
    }

    private void stopLeaderMotor() {
        appliedSysIdVoltage = 0.0;
        leaderMotor.stopMotor();
    }
}
