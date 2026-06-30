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
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final CANBus intakeCanBus = new CANBus(IntakeConstants.kMotorCanBus);

    private final TalonFX deployMotor = new TalonFX(
        IntakeConstants.kDeployMotorCanId,
        intakeCanBus
    );
    private final TalonFX leftRollerMotor = new TalonFX(
        IntakeConstants.kLeftRollerMotorCanId,
        intakeCanBus
    );
    private final TalonFX rightRollerMotor = new TalonFX(
        IntakeConstants.kRightRollerMotorCanId,
        intakeCanBus
    );

    private double targetPositionMotorRotations;
    private double appliedDeployMotorOutput;
    private double appliedRollerOutput;
    private double homingStartTimestampSeconds;
    private double highCurrentStartTimestampSeconds = Double.NaN;
    private boolean targetDeployed;
    private boolean positionControlActive;
    private boolean homing;
    private boolean homed;

    public Intake() {
        deployMotor.setNeutralMode(NeutralModeValue.Coast);
        leftRollerMotor.setNeutralMode(NeutralModeValue.Coast);
        rightRollerMotor.setNeutralMode(NeutralModeValue.Coast);

        applyDeployOperatingCurrentLimits();

        TalonFXConfiguration rollerConfiguration = new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(IntakeConstants.kRollerSupplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(IntakeConstants.kRollerStatorCurrentLimitAmps)
                .withStatorCurrentLimitEnable(true)
        );
        leftRollerMotor.getConfigurator().apply(rollerConfiguration);
        rightRollerMotor.getConfigurator().apply(rollerConfiguration);

        rightRollerMotor.setControl(
            new Follower(
                IntakeConstants.kLeftRollerMotorCanId,
                getRightRollerMotorAlignment()
            )
        );

        targetPositionMotorRotations = getIntakePositionMotorRotations();
    }

    public double getIntakePositionMotorRotations() {
        return IntakeConstants.kDeployPositionSensorSign * getRawDeployMotorPositionRotations();
    }

    public double getTargetPositionMotorRotations() {
        return targetPositionMotorRotations;
    }

    public double getDeployStatorCurrentAmps() {
        return deployMotor.getStatorCurrent().getValueAsDouble();
    }

    public boolean isHoming() {
        return homing;
    }

    public boolean isHomed() {
        return homed;
    }

    public boolean isTargetDeployed() {
        return targetDeployed;
    }

    public boolean isPositionControlAllowed() {
        return homed;
    }

    public boolean isPositionControlActive() {
        return positionControlActive;
    }

    public void moveToStowedSetpoint() {
        if (!isPositionControlAllowed()) {
            targetPositionMotorRotations = getIntakePositionMotorRotations();
            positionControlActive = false;
            return;
        }

        homing = false;
        targetDeployed = false;
        targetPositionMotorRotations = 0.0;
        positionControlActive = true;
    }

    public void moveToDeployedSetpoint() {
        if (!isPositionControlAllowed()) {
            targetPositionMotorRotations = getIntakePositionMotorRotations();
            positionControlActive = false;
            return;
        }

        homing = false;
        targetDeployed = true;
        targetPositionMotorRotations = IntakeConstants.kDeployedSetpointMotorRotations;
        positionControlActive = true;
    }

    public void startHoming() {
        applyDeployHomingCurrentLimits();
        homing = true;
        homed = false;
        targetDeployed = false;
        positionControlActive = false;
        homingStartTimestampSeconds = Timer.getFPGATimestamp();
        highCurrentStartTimestampSeconds = Double.NaN;
    }

    public void runRollersIn() {
        setRollerOutput(IntakeConstants.kRollerMotorOutput);
    }

    public void runRollersOut() {
        setRollerOutput(-IntakeConstants.kRollerMotorOutput);
    }

    public void stopRollers() {
        setRollerOutput(0.0);
    }

    public void updateControlAndTelemetry() {
        if (DriverStation.isDisabled()) {
            if (homing) {
                applyDeployOperatingCurrentLimits();
            }
            homing = false;
            positionControlActive = false;
            setDeployMotorOutput(0.0);
            stopRollers();
        } else if (homing) {
            updateHomingControl();
        } else if (!isPositionControlAllowed()) {
            targetPositionMotorRotations = getIntakePositionMotorRotations();
            positionControlActive = false;
            setDeployMotorOutput(0.0);
        } else if (!isPositionControlActive()) {
            setDeployMotorOutput(0.0);
        } else {
            updatePositionControl();
        }

        SmartDashboard.putNumber("Intake/PositionMotorRotations", getIntakePositionMotorRotations());
        SmartDashboard.putNumber("Intake/TargetMotorRotations", getTargetPositionMotorRotations());
        SmartDashboard.putNumber("Intake/DeployMotorOutput", appliedDeployMotorOutput);
        SmartDashboard.putNumber("Intake/RollerOutput", appliedRollerOutput);
        SmartDashboard.putBoolean("Intake/Homing", isHoming());
        SmartDashboard.putBoolean("Intake/Homed", isHomed());
    }

    private void updateHomingControl() {
        double nowSeconds = Timer.getFPGATimestamp();
        setDeployMotorOutput(IntakeConstants.kHomingMotorOutput);

        if (nowSeconds - homingStartTimestampSeconds < IntakeConstants.kHomingMinRunTimeSeconds) {
            return;
        }

        if (getDeployStatorCurrentAmps() < IntakeConstants.kHomingCurrentThresholdAmps) {
            highCurrentStartTimestampSeconds = Double.NaN;
            return;
        }

        if (Double.isNaN(highCurrentStartTimestampSeconds)) {
            highCurrentStartTimestampSeconds = nowSeconds;
            return;
        }

        if (nowSeconds - highCurrentStartTimestampSeconds >= IntakeConstants.kHomingCurrentDebounceSeconds) {
            deployMotor.setPosition(0.0);
            targetPositionMotorRotations = 0.0;
            homing = false;
            homed = true;
            positionControlActive = false;
            applyDeployOperatingCurrentLimits();
            setDeployMotorOutput(0.0);
        }
    }

    private void updatePositionControl() {
        double positionErrorMotorRotations = getPositionErrorMotorRotations();
        if (Math.abs(positionErrorMotorRotations) <= IntakeConstants.kPositionToleranceMotorRotations) {
            positionControlActive = false;
            setDeployMotorOutput(0.0);
            return;
        }

        double motorOutput = IntakeConstants.kPositionMotorOutputSign
            * IntakeConstants.kPositionKp
            * positionErrorMotorRotations;

        setDeployMotorOutput(
            MathUtil.clamp(
                motorOutput,
                -IntakeConstants.kMaxPositionMotorOutput,
                IntakeConstants.kMaxPositionMotorOutput
            )
        );
    }

    private double getPositionErrorMotorRotations() {
        return targetPositionMotorRotations - getIntakePositionMotorRotations();
    }

    private double getRawDeployMotorPositionRotations() {
        return deployMotor.getPosition().getValueAsDouble();
    }

    private MotorAlignmentValue getRightRollerMotorAlignment() {
        return IntakeConstants.kRightRollerOpposesLeft
            ? MotorAlignmentValue.Opposed
            : MotorAlignmentValue.Aligned;
    }

    private void applyDeployHomingCurrentLimits() {
        applyDeployCurrentLimits(
            IntakeConstants.kDeployHomingSupplyCurrentLimitAmps,
            IntakeConstants.kDeployHomingStatorCurrentLimitAmps
        );
    }

    private void applyDeployOperatingCurrentLimits() {
        applyDeployCurrentLimits(
            IntakeConstants.kDeploySupplyCurrentLimitAmps,
            IntakeConstants.kDeployStatorCurrentLimitAmps
        );
    }

    private void applyDeployCurrentLimits(double supplyCurrentLimitAmps, double statorCurrentLimitAmps) {
        deployMotor.getConfigurator().apply(
            new TalonFXConfiguration().withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(supplyCurrentLimitAmps)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(statorCurrentLimitAmps)
                    .withStatorCurrentLimitEnable(true)
            )
        );
    }

    private void setDeployMotorOutput(double motorOutput) {
        appliedDeployMotorOutput = motorOutput;
        deployMotor.set(motorOutput);
    }

    private void setRollerOutput(double rollerOutput) {
        appliedRollerOutput = rollerOutput;
        leftRollerMotor.set(IntakeConstants.kRollerMotorOutputSign * rollerOutput);
    }
}
