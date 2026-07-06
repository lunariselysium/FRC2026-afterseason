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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TurretPitchConstants;

public class TurretPitch {
    private final CANBus pitchCanBus = new CANBus(TurretPitchConstants.kPitchMotorCanBus);

    private final TalonFX pitchMotor = new TalonFX(
        TurretPitchConstants.kPitchMotorCanId,
        pitchCanBus
    );

    private double targetPitchDegrees;
    private double profiledPitchDegrees;
    private double profiledPitchVelocityDegreesPerSecond;
    private double lastProfileUpdateTimeSeconds;
    private double appliedPitchMotorOutput;
    private double appliedSysIdMechanismVoltage;
    private double homingStartTimestampSeconds;
    private double highCurrentStartTimestampSeconds = Double.NaN;
    private boolean homing;
    private boolean homed;
    private boolean homingTimedOut;
    private boolean sysIdActive;
    private boolean reportedSysIdNotHomedWarning;

    public TurretPitch() {
        pitchMotor.getConfigurator().apply(
            new TalonFXConfiguration().withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(TurretPitchConstants.kPitchSupplyCurrentLimitAmps)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(TurretPitchConstants.kPitchStatorCurrentLimitAmps)
                    .withStatorCurrentLimitEnable(true)
            )
        );
        pitchMotor.setNeutralMode(NeutralModeValue.Brake);

        targetPitchDegrees = clampPitchToTravelWindow(getPitchDegrees());
        profiledPitchDegrees = targetPitchDegrees;
        lastProfileUpdateTimeSeconds = Timer.getFPGATimestamp();
    }

    public double getPitchDegrees() {
        return TurretPitchConstants.kPitchPositionSign
            * pitchMotor.getPosition().getValueAsDouble()
            * TurretPitchConstants.kPitchDegreesPerMotorRotation;
    }

    public double getTargetPitchDegrees() {
        return targetPitchDegrees;
    }

    public double getProfiledPitchDegrees() {
        return profiledPitchDegrees;
    }

    public double getPitchErrorDegrees() {
        return profiledPitchDegrees - getPitchDegrees();
    }

    public double getAppliedMotorOutput() {
        return appliedPitchMotorOutput;
    }

    public double getPitchVelocityDegreesPerSecond() {
        return TurretPitchConstants.kPitchPositionSign
            * pitchMotor.getVelocity().getValueAsDouble()
            * TurretPitchConstants.kPitchDegreesPerMotorRotation;
    }

    public double getAppliedSysIdMechanismVoltage() {
        return appliedSysIdMechanismVoltage;
    }

    public double getMeasuredMechanismVoltage() {
        return TurretPitchConstants.kPitchMotorOutputSign
            * pitchMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getStatorCurrentAmps() {
        return pitchMotor.getStatorCurrent().getValueAsDouble();
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

    public boolean isSysIdActive() {
        return sysIdActive;
    }

    public boolean isAtTarget() {
        return homed
            && Math.abs(targetPitchDegrees - getPitchDegrees())
                <= TurretPitchConstants.kPitchToleranceDegrees;
    }

    public void stepTargetUp() {
        setTargetPitchDegrees(targetPitchDegrees + TurretPitchConstants.kTargetPitchStepDegrees);
    }

    public void stepTargetDown() {
        setTargetPitchDegrees(targetPitchDegrees - TurretPitchConstants.kTargetPitchStepDegrees);
    }

    public void startHoming() {
        homing = true;
        homed = false;
        homingTimedOut = false;
        sysIdActive = false;
        homingStartTimestampSeconds = Timer.getFPGATimestamp();
        highCurrentStartTimestampSeconds = Double.NaN;
    }

    public void prepareSysId() {
        homing = false;
        sysIdActive = false;
        reportedSysIdNotHomedWarning = false;
        holdCurrentPitch();
        setMechanismVoltage(0.0);
    }

    public void runSysIdVoltage(double mechanismVoltage) {
        if (Math.abs(mechanismVoltage) < 1.0e-6) {
            stopSysId();
            return;
        }

        if (!isHomed()) {
            reportSysIdNotHomedWarningOnce();
            stopSysId();
            return;
        }

        homing = false;
        sysIdActive = true;
        setMechanismVoltage(mechanismVoltage);
    }

    public void stopSysId() {
        sysIdActive = false;
        holdCurrentPitch();
        setMechanismVoltage(0.0);
    }

    public void updateControl() {
        if (DriverStation.isDisabled()) {
            homing = false;
            sysIdActive = false;
            setMechanismOutput(0.0);
        } else if (sysIdActive) {
            homing = false;
        } else if (homing) {
            updateHomingControl();
        } else if (homed) {
            updateMotionProfile();
            updatePitchController();
        } else {
            setMechanismOutput(0.0);
            holdCurrentPitch();
        }
    }

    private void updateHomingControl() {
        double nowSeconds = Timer.getFPGATimestamp();
        double homingElapsedSeconds = nowSeconds - homingStartTimestampSeconds;

        if (homingElapsedSeconds >= TurretPitchConstants.kPitchHomingTimeoutSeconds) {
            failHomingTimeout();
            return;
        }

        setMechanismOutput(-TurretPitchConstants.kPitchHomingMotorOutput, false);

        if (homingElapsedSeconds < TurretPitchConstants.kPitchHomingMinRunTimeSeconds) {
            return;
        }

        if (getStatorCurrentAmps() < TurretPitchConstants.kPitchHomingCurrentThresholdAmps) {
            highCurrentStartTimestampSeconds = Double.NaN;
            return;
        }

        if (Double.isNaN(highCurrentStartTimestampSeconds)) {
            highCurrentStartTimestampSeconds = nowSeconds;
            return;
        }

        if (nowSeconds - highCurrentStartTimestampSeconds
            >= TurretPitchConstants.kPitchHomingCurrentDebounceSeconds) {
            pitchMotor.setPosition(0.0);
            targetPitchDegrees = TurretPitchConstants.kMinPitchDegrees;
            profiledPitchDegrees = TurretPitchConstants.kMinPitchDegrees;
            profiledPitchVelocityDegreesPerSecond = 0.0;
            homing = false;
            homed = true;
            setMechanismOutput(0.0);
        }
    }

    private void failHomingTimeout() {
        homing = false;
        homed = false;
        homingTimedOut = true;
        targetPitchDegrees = clampPitchToTravelWindow(getPitchDegrees());
        profiledPitchDegrees = targetPitchDegrees;
        profiledPitchVelocityDegreesPerSecond = 0.0;
        lastProfileUpdateTimeSeconds = Timer.getFPGATimestamp();
        setMechanismOutput(0.0);

        DriverStation.reportWarning(
            "Turret pitch homing timed out after "
                + TurretPitchConstants.kPitchHomingTimeoutSeconds
                + " seconds; stopping pitch motor.",
            false
        );
    }

    public void setTargetPitchDegrees(double requestedPitchDegrees) {
        targetPitchDegrees = clampPitchToTravelWindow(requestedPitchDegrees);
    }

    private void holdCurrentPitch() {
        targetPitchDegrees = clampPitchToTravelWindow(getPitchDegrees());
        profiledPitchDegrees = targetPitchDegrees;
        profiledPitchVelocityDegreesPerSecond = 0.0;
        lastProfileUpdateTimeSeconds = Timer.getFPGATimestamp();
    }

    private double clampPitchToTravelWindow(double pitchDegrees) {
        return MathUtil.clamp(
            pitchDegrees,
            TurretPitchConstants.kMinPitchDegrees,
            TurretPitchConstants.kMaxPitchDegrees
        );
    }

    private void updateMotionProfile() {
        double nowSeconds = Timer.getFPGATimestamp();
        double dtSeconds = MathUtil.clamp(nowSeconds - lastProfileUpdateTimeSeconds, 0.0, 0.05);
        lastProfileUpdateTimeSeconds = nowSeconds;

        double profileErrorDegrees = targetPitchDegrees - profiledPitchDegrees;
        if (dtSeconds <= 0.0) {
            return;
        }

        if (Math.abs(profileErrorDegrees) <= TurretPitchConstants.kPitchToleranceDegrees
            && Math.abs(profiledPitchVelocityDegreesPerSecond) <= 1.0) {
            profiledPitchDegrees = targetPitchDegrees;
            profiledPitchVelocityDegreesPerSecond = 0.0;
            return;
        }

        double desiredVelocityMagnitude = Math.min(
            TurretPitchConstants.kMaxPitchVelocityDegreesPerSecond,
            Math.sqrt(
                2.0
                    * TurretPitchConstants.kMaxPitchAccelerationDegreesPerSecondSquared
                    * Math.abs(profileErrorDegrees)
            )
        );
        double desiredVelocityDegreesPerSecond = Math.copySign(
            desiredVelocityMagnitude,
            profileErrorDegrees
        );

        double maxVelocityStep = TurretPitchConstants.kMaxPitchAccelerationDegreesPerSecondSquared
            * dtSeconds;
        profiledPitchVelocityDegreesPerSecond = MathUtil.clamp(
            desiredVelocityDegreesPerSecond,
            profiledPitchVelocityDegreesPerSecond - maxVelocityStep,
            profiledPitchVelocityDegreesPerSecond + maxVelocityStep
        );

        double nextProfiledPitchDegrees = profiledPitchDegrees
            + profiledPitchVelocityDegreesPerSecond * dtSeconds;

        if (Math.signum(targetPitchDegrees - nextProfiledPitchDegrees)
            != Math.signum(profileErrorDegrees)) {
            profiledPitchDegrees = targetPitchDegrees;
            profiledPitchVelocityDegreesPerSecond = 0.0;
            return;
        }

        profiledPitchDegrees = clampPitchToTravelWindow(nextProfiledPitchDegrees);
    }

    private void updatePitchController() {
        double pitchErrorDegrees = getPitchErrorDegrees();
        if (Math.abs(pitchErrorDegrees) <= TurretPitchConstants.kPitchToleranceDegrees) {
            setMechanismOutput(0.0);
            return;
        }

        double gravityFeedforward = TurretPitchConstants.kPitchKg
            * Math.cos(Math.toRadians(getPitchDegrees() + TurretPitchConstants.kPitchGravityOffsetDegrees));
        double mechanismOutput = gravityFeedforward
            + TurretPitchConstants.kPitchKv * profiledPitchVelocityDegreesPerSecond
            + TurretPitchConstants.kPitchKp * pitchErrorDegrees;

        setMechanismOutput(
            MathUtil.clamp(
                mechanismOutput,
                -TurretPitchConstants.kMaxPitchMotorOutput,
                TurretPitchConstants.kMaxPitchMotorOutput
            )
        );
    }

    private void setMechanismOutput(double mechanismOutput) {
        setMechanismOutput(mechanismOutput, true);
    }

    private void setMechanismOutput(double mechanismOutput, boolean shouldApplyTravelLimits) {
        double safeMechanismOutput = shouldApplyTravelLimits
            ? applyTravelLimits(mechanismOutput)
            : mechanismOutput;

        appliedPitchMotorOutput = TurretPitchConstants.kPitchMotorOutputSign * safeMechanismOutput;
        appliedSysIdMechanismVoltage = 0.0;
        pitchMotor.set(appliedPitchMotorOutput);
    }

    private double applyTravelLimits(double mechanismOutput) {
        double currentPitchDegrees = getPitchDegrees();

        if (currentPitchDegrees >= TurretPitchConstants.kMaxPitchDegrees && mechanismOutput > 0.0) {
            return 0.0;
        }

        if (currentPitchDegrees <= TurretPitchConstants.kMinPitchDegrees && mechanismOutput < 0.0) {
            return 0.0;
        }

        return mechanismOutput;
    }

    private void setMechanismVoltage(double mechanismVoltage) {
        double safeMechanismVoltage = applyTravelLimits(mechanismVoltage);

        appliedPitchMotorOutput = 0.0;
        appliedSysIdMechanismVoltage = safeMechanismVoltage;
        pitchMotor.setVoltage(
            TurretPitchConstants.kPitchMotorOutputSign * safeMechanismVoltage
        );
    }

    private void reportSysIdNotHomedWarningOnce() {
        if (reportedSysIdNotHomedWarning) {
            return;
        }

        DriverStation.reportWarning(
            "Turret pitch SysId requested before pitch homing; pitch motor will stay stopped.",
            false
        );
        reportedSysIdNotHomedWarning = true;
    }
}
