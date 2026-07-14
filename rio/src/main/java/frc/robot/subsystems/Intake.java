// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final CANBus intakeCanBus = new CANBus(IntakeConstants.kMotorCanBus);

    private final TalonFX deployMotor = new TalonFX(
        IntakeConstants.kDeployMotorCanId,
        intakeCanBus
    );
    private final TalonFX rightRollerMotor = new TalonFX(
        IntakeConstants.kRightRollerMotorCanId,
        intakeCanBus
    );
    private final Optional<TalonFX> leftRollerMotor = IntakeConstants.kLeftRollerMotorPresent
        ? Optional.of(new TalonFX(IntakeConstants.kLeftRollerMotorCanId, intakeCanBus))
        : Optional.empty();
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);

    private boolean autoScoreRetractionMotionProfileActive;
    private final PendingIntakeDeploy pendingDeploy = new PendingIntakeDeploy();
    private double targetPositionMotorRotations;
    private double appliedDeployMotorOutput;
    private double appliedDeployMechanismVoltage;
    private double appliedClosedLoopFeedForwardVolts;
    private double appliedRollerOutput;
    private double homingStartTimestampSeconds;
    private double highCurrentStartTimestampSeconds = Double.NaN;
    private boolean deployedHardstopCurrentHigh;
    private boolean deployedHardstopCaptured;
    private boolean targetDeployed;
    private boolean positionControlActive;
    private boolean homing;
    private boolean homed;
    private boolean homingTimedOut;
    private boolean sysIdActive;
    private boolean reportedSysIdNotHomedWarning;

    private final SysIdRoutine deploySysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(IntakeConstants.kSysIdQuasistaticRampRateVoltsPerSecond).per(Second),
            Volts.of(IntakeConstants.kSysIdDynamicStepVolts),
            Seconds.of(IntakeConstants.kSysIdTimeoutSeconds)
        ),
        new SysIdRoutine.Mechanism(
            this::runDeploySysIdVoltage,
            log -> log.motor("intake-deploy")
                .voltage(Volts.of(appliedDeployMechanismVoltage))
                .angularPosition(Rotations.of(getIntakePositionMotorRotations()))
                .angularVelocity(RotationsPerSecond.of(getIntakeVelocityMotorRotationsPerSecond()))
                .current(Amps.of(getDeployStatorCurrentAmps())),
            this,
            "IntakeDeploy"
        )
    );

    public Intake() {
        deployMotor.setNeutralMode(NeutralModeValue.Coast);
        rightRollerMotor.setNeutralMode(NeutralModeValue.Coast);
        leftRollerMotor.ifPresent(motor -> motor.setNeutralMode(NeutralModeValue.Coast));

        applyDeployOperatingCurrentLimits();
        applyDeployPositionControlConfig();

        TalonFXConfiguration rollerConfiguration = new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(IntakeConstants.kRollerSupplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(IntakeConstants.kRollerStatorCurrentLimitAmps)
                .withStatorCurrentLimitEnable(true)
        );
        rightRollerMotor.getConfigurator().apply(rollerConfiguration);

        leftRollerMotor.ifPresent(motor -> {
            motor.getConfigurator().apply(rollerConfiguration);
            motor.setControl(
                new Follower(
                    IntakeConstants.kRightRollerMotorCanId,
                    getLeftRollerMotorAlignment()
                )
            );
        });

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

    public double getDeployMechanismVoltage() {
        return IntakeConstants.kPositionMotorOutputSign * deployMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getIntakeVelocityMotorRotationsPerSecond() {
        return IntakeConstants.kDeployPositionSensorSign * deployMotor.getVelocity().getValueAsDouble();
    }

    public boolean isHoming() {
        return homing;
    }

    public boolean isHomed() {
        return homed;
    }

    public boolean didHomingTimeOut() {
        return homingTimedOut;
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

    public boolean isClosedLoopAtTarget() {
        return Math.abs(getClosedLoopErrorMotorRotations())
            <= IntakeConstants.kPositionToleranceMotorRotations;
    }

    public boolean isSysIdActive() {
        return sysIdActive;
    }

    public Command sysIdQuasistatic(Direction direction) {
        return runOnce(this::prepareDeploySysId).andThen(deploySysIdRoutine.quasistatic(direction));
    }

    public Command sysIdDynamic(Direction direction) {
        return runOnce(this::prepareDeploySysId).andThen(deploySysIdRoutine.dynamic(direction));
    }

    public void moveToStowedSetpoint() {
        pendingDeploy.clear();
        if (!isPositionControlAllowed()) {
            targetPositionMotorRotations = getIntakePositionMotorRotations();
            positionControlActive = false;
            return;
        }

        homing = false;
        sysIdActive = false;
        useNormalDeployMotionProfile();
        targetDeployed = false;
        resetDeployedHardstopCapture();
        targetPositionMotorRotations = 0.0;
        positionControlActive = true;
    }

    public void moveToDeployedSetpoint() {
        if (!isPositionControlAllowed()) {
            pendingDeploy.request();
            return;
        }

        pendingDeploy.clear();
        homing = false;
        sysIdActive = false;
        useNormalDeployMotionProfile();
        targetDeployed = true;
        resetDeployedHardstopCapture();
        targetPositionMotorRotations = IntakeConstants.kDeployedSetpointMotorRotations;
        positionControlActive = true;
    }

    public void moveToAutoScoreRetractionSetpoint() {
        moveToAutoScoreSetpoint(IntakeConstants.kAutoScoreRetractionSetpointMotorRotations);
    }

    public void moveToAutoScoreSeventyPercentDeployedSetpoint() {
        moveToAutoScoreSetpoint(IntakeConstants.kAutoScoreOscillationSetpointMotorRotations);
    }

    private void moveToAutoScoreSetpoint(double targetMotorRotations) {
        pendingDeploy.clear();
        if (!isPositionControlAllowed()) {
            targetPositionMotorRotations = getIntakePositionMotorRotations();
            positionControlActive = false;
            return;
        }

        homing = false;
        sysIdActive = false;
        useAutoScoreRetractionMotionProfile();
        targetDeployed = false;
        resetDeployedHardstopCapture();
        targetPositionMotorRotations = targetMotorRotations;
        positionControlActive = true;
    }

    public void startHoming() {
        applyDeployHomingCurrentLimits();
        homing = true;
        homed = false;
        homingTimedOut = false;
        sysIdActive = false;
        targetDeployed = false;
        positionControlActive = false;
        resetDeployedHardstopCapture();
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
            pendingDeploy.clear();
            if (homing) {
                applyDeployOperatingCurrentLimits();
            }
            homing = false;
            positionControlActive = false;
            sysIdActive = false;
            resetDeployedHardstopCapture();
            setDeployMotorOutput(0.0);
            stopRollers();
        } else if (isSysIdActive()) {
            positionControlActive = false;
            homing = false;
            resetDeployedHardstopCapture();
        } else if (homing) {
            updateHomingControl();
        } else if (pendingDeploy.consumeWhenAllowed(isPositionControlAllowed())) {
            moveToDeployedSetpoint();
            updatePositionControl();
        } else if (!isPositionControlAllowed()) {
            targetPositionMotorRotations = getIntakePositionMotorRotations();
            positionControlActive = false;
            resetDeployedHardstopCapture();
            setDeployMotorOutput(0.0);
        } else if (!isPositionControlActive()) {
            deployedHardstopCurrentHigh = false;
            setDeployMotorOutput(0.0);
        } else {
            updatePositionControl();
        }

        SmartDashboard.putNumber("Intake/PositionMotorRotations", getIntakePositionMotorRotations());
        SmartDashboard.putNumber("Intake/TargetMotorRotations", getTargetPositionMotorRotations());
        SmartDashboard.putNumber("Intake/DeployMotorOutput", appliedDeployMotorOutput);
        SmartDashboard.putNumber("Intake/DeployMechanismVoltage", appliedDeployMechanismVoltage);
        SmartDashboard.putNumber("Intake/ClosedLoopFeedForwardVolts", appliedClosedLoopFeedForwardVolts);
        SmartDashboard.putNumber("Intake/MeasuredMechanismVoltage", getDeployMechanismVoltage());
        SmartDashboard.putNumber("Intake/VelocityMotorRotationsPerSecond", getIntakeVelocityMotorRotationsPerSecond());
        SmartDashboard.putNumber("Intake/ClosedLoopReferenceMotorRotations", getClosedLoopReferenceMotorRotations());
        SmartDashboard.putNumber("Intake/ClosedLoopErrorMotorRotations", getClosedLoopErrorMotorRotations());
        SmartDashboard.putNumber("Intake/ClosedLoopOutput", getClosedLoopOutput());
        SmartDashboard.putBoolean("Intake/AtTarget", isAtTargetPosition());
        SmartDashboard.putBoolean("Intake/ClosedLoopAtTarget", isClosedLoopAtTarget());
        SmartDashboard.putNumber("Intake/RollerOutput", appliedRollerOutput);
        SmartDashboard.putBoolean("Intake/LeftRollerMotorPresent", IntakeConstants.kLeftRollerMotorPresent);
        SmartDashboard.putBoolean("Intake/Homing", isHoming());
        SmartDashboard.putBoolean("Intake/Homed", isHomed());
        SmartDashboard.putBoolean("Intake/TargetDeployed", isTargetDeployed());
        SmartDashboard.putBoolean("Intake/PositionControlActive", isPositionControlActive());
        SmartDashboard.putBoolean("Intake/HomingTimedOut", didHomingTimeOut());
        SmartDashboard.putBoolean("Intake/SysIdActive", isSysIdActive());
        SmartDashboard.putBoolean(
            "Intake/AutoScoreRetractionMotionProfileActive",
            autoScoreRetractionMotionProfileActive
        );
        SmartDashboard.putBoolean("Intake/DeployedHardstopCurrentHigh", deployedHardstopCurrentHigh);
        SmartDashboard.putBoolean("Intake/DeployedHardstopCaptured", deployedHardstopCaptured);
    }

    private void updateHomingControl() {
        double nowSeconds = Timer.getFPGATimestamp();
        double homingElapsedSeconds = nowSeconds - homingStartTimestampSeconds;

        if (homingElapsedSeconds >= IntakeConstants.kHomingTimeoutSeconds) {
            failHomingTimeout();
            return;
        }

        setDeployMotorOutput(IntakeConstants.kHomingMotorOutput);

        if (homingElapsedSeconds < IntakeConstants.kHomingMinRunTimeSeconds) {
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

    private void failHomingTimeout() {
        pendingDeploy.clear();
        homing = false;
        homed = false;
        homingTimedOut = true;
        positionControlActive = false;
        targetPositionMotorRotations = getIntakePositionMotorRotations();
        applyDeployOperatingCurrentLimits();
        setDeployMotorOutput(0.0);

        DriverStation.reportWarning(
            "Intake deploy homing timed out after "
                + IntakeConstants.kHomingTimeoutSeconds
                + " seconds; stopping deploy motor.",
            false
        );
    }

    private void prepareDeploySysId() {
        pendingDeploy.clear();
        applyDeployOperatingCurrentLimits();
        homing = false;
        positionControlActive = false;
        sysIdActive = false;
        reportedSysIdNotHomedWarning = false;
        stopRollers();
        setDeployMotorVoltage(0.0);
    }

    private void runDeploySysIdVoltage(Voltage voltage) {
        double requestedMechanismVoltage = voltage.in(Volts);
        if (Math.abs(requestedMechanismVoltage) < 1.0e-6) {
            sysIdActive = false;
            setDeployMotorVoltage(0.0);
            return;
        }

        sysIdActive = true;
        if (!isHomed()) {
            reportSysIdNotHomedWarningOnce();
            setDeployMotorVoltage(0.0);
            return;
        }

        setDeployMotorVoltage(applySysIdTravelLimits(requestedMechanismVoltage));
    }

    private double applySysIdTravelLimits(double mechanismVoltage) {
        double intakePositionMotorRotations = getIntakePositionMotorRotations();

        if (intakePositionMotorRotations >= IntakeConstants.kDeployedSetpointMotorRotations
            && mechanismVoltage > 0.0) {
            return 0.0;
        }

        if (intakePositionMotorRotations <= 0.0 && mechanismVoltage < 0.0) {
            return 0.0;
        }

        return mechanismVoltage;
    }

    private void reportSysIdNotHomedWarningOnce() {
        if (reportedSysIdNotHomedWarning) {
            return;
        }

        DriverStation.reportWarning(
            "Intake deploy SysId requested before homing; deploy motor will stay stopped.",
            false
        );
        reportedSysIdNotHomedWarning = true;
    }

    private void updatePositionControl() {
        targetPositionMotorRotations = clampPositionToTravelWindow(targetPositionMotorRotations);
        if (shouldCaptureDeployedHardstop()) {
            captureDeployedHardstopPosition();
            return;
        }

        setDeployMotionMagicTarget(targetPositionMotorRotations);
    }

    private boolean shouldCaptureDeployedHardstop() {
        if (!targetDeployed || !isNearDeployedHardstopCaptureWindow()) {
            deployedHardstopCurrentHigh = false;
            return false;
        }

        boolean currentHigh = getDeployStatorCurrentAmps()
            >= IntakeConstants.kDeployedHardstopCaptureCurrentThresholdAmps;
        boolean currentRisingEdge = currentHigh && !deployedHardstopCurrentHigh;
        deployedHardstopCurrentHigh = currentHigh;
        return currentRisingEdge;
    }

    private boolean isNearDeployedHardstopCaptureWindow() {
        return getIntakePositionMotorRotations()
            >= IntakeConstants.kDeployedSetpointMotorRotations
                - IntakeConstants.kDeployedHardstopCaptureWindowMotorRotations;
    }

    private void captureDeployedHardstopPosition() {
        deployMotor.setPosition(
            getRawDeployMotorTargetRotations(IntakeConstants.kDeployedSetpointMotorRotations)
        );
        targetPositionMotorRotations = IntakeConstants.kDeployedSetpointMotorRotations;
        positionControlActive = false;
        deployedHardstopCaptured = true;
        setDeployMotorOutput(0.0);
    }

    private void resetDeployedHardstopCapture() {
        deployedHardstopCurrentHigh = false;
        deployedHardstopCaptured = false;
    }

    private boolean isAtTargetPosition() {
        return Math.abs(getPositionErrorMotorRotations())
            <= IntakeConstants.kPositionToleranceMotorRotations;
    }

    private double getPositionErrorMotorRotations() {
        return targetPositionMotorRotations - getIntakePositionMotorRotations();
    }

    private double clampPositionToTravelWindow(double positionMotorRotations) {
        return Math.max(
            0.0,
            Math.min(IntakeConstants.kDeployedSetpointMotorRotations, positionMotorRotations)
        );
    }

    private double getRawDeployMotorPositionRotations() {
        return deployMotor.getPosition().getValueAsDouble();
    }

    private double getClosedLoopReferenceMotorRotations() {
        return IntakeConstants.kDeployPositionSensorSign
            * deployMotor.getClosedLoopReference().getValueAsDouble();
    }

    private double getClosedLoopErrorMotorRotations() {
        return IntakeConstants.kDeployPositionSensorSign
            * deployMotor.getClosedLoopError().getValueAsDouble();
    }

    private double getClosedLoopOutput() {
        return deployMotor.getClosedLoopOutput().getValueAsDouble();
    }

    private double getRawDeployMotorTargetRotations(double targetMechanismMotorRotations) {
        return IntakeConstants.kDeployPositionSensorSign * targetMechanismMotorRotations;
    }

    private MotorAlignmentValue getLeftRollerMotorAlignment() {
        return IntakeConstants.kLeftRollerOpposesRight
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
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(supplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(statorCurrentLimitAmps)
                .withStatorCurrentLimitEnable(true)
        );
    }

    private void applyDeployPositionControlConfig() {
        deployMotor.getConfigurator().apply(
            new Slot0Configs()
                .withKP(IntakeConstants.kPositionClosedLoopKp)
                .withKD(IntakeConstants.kPositionClosedLoopKd)
                .withKS(IntakeConstants.kPositionClosedLoopKs)
                .withKV(IntakeConstants.kPositionClosedLoopKv)
                .withKA(IntakeConstants.kPositionClosedLoopKa)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        );
        applyDeployMotionMagicConfig(
            IntakeConstants.kMotionMagicCruiseVelocityMotorRotationsPerSecond,
            IntakeConstants.kMotionMagicAccelerationMotorRotationsPerSecondSquared
        );
    }

    private void useNormalDeployMotionProfile() {
        if (!autoScoreRetractionMotionProfileActive) {
            return;
        }

        applyDeployMotionMagicConfig(
            IntakeConstants.kMotionMagicCruiseVelocityMotorRotationsPerSecond,
            IntakeConstants.kMotionMagicAccelerationMotorRotationsPerSecondSquared
        );
        autoScoreRetractionMotionProfileActive = false;
    }

    private void useAutoScoreRetractionMotionProfile() {
        if (autoScoreRetractionMotionProfileActive) {
            return;
        }

        applyDeployMotionMagicConfig(
            IntakeConstants.kAutoScoreRetractionCruiseVelocityMotorRotationsPerSecond,
            IntakeConstants.kAutoScoreRetractionAccelerationMotorRotationsPerSecondSquared
        );
        autoScoreRetractionMotionProfileActive = true;
    }

    private void applyDeployMotionMagicConfig(
        double cruiseVelocityMotorRotationsPerSecond,
        double accelerationMotorRotationsPerSecondSquared
    ) {
        deployMotor.getConfigurator().apply(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(
                    cruiseVelocityMotorRotationsPerSecond
                )
                .withMotionMagicAcceleration(
                    accelerationMotorRotationsPerSecondSquared
                )
        );
    }

    private void setDeployMotorOutput(double motorOutput) {
        appliedDeployMotorOutput = motorOutput;
        appliedDeployMechanismVoltage = 0.0;
        appliedClosedLoopFeedForwardVolts = 0.0;
        deployMotor.set(motorOutput);
    }

    private void setDeployMotorVoltage(double mechanismVoltage) {
        appliedDeployMechanismVoltage = mechanismVoltage;
        appliedDeployMotorOutput = 0.0;
        appliedClosedLoopFeedForwardVolts = 0.0;
        deployMotor.setVoltage(IntakeConstants.kPositionMotorOutputSign * mechanismVoltage);
    }

    private void setDeployMotionMagicTarget(double targetMechanismMotorRotations) {
        double feedForwardVolts = getRawDeployMotorFeedForwardVolts(targetMechanismMotorRotations);
        appliedDeployMotorOutput = 0.0;
        appliedDeployMechanismVoltage = 0.0;
        appliedClosedLoopFeedForwardVolts = feedForwardVolts;
        deployMotor.setControl(
            positionRequest
                .withPosition(getRawDeployMotorTargetRotations(targetMechanismMotorRotations))
                .withFeedForward(feedForwardVolts)
        );
    }

    private double getRawDeployMotorFeedForwardVolts(double targetMechanismMotorRotations) {
        if (targetMechanismMotorRotations <= 0.0) {
            return IntakeConstants.kStowAssistFeedForwardVolts;
        }

        return 0.0;
    }

    private void setRollerOutput(double rollerOutput) {
        appliedRollerOutput = rollerOutput;
        rightRollerMotor.set(IntakeConstants.kRightRollerMotorOutputSign * rollerOutput);
    }
}
