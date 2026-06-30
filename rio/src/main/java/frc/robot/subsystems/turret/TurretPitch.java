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
    private double homingStartTimestampSeconds;
    private double highCurrentStartTimestampSeconds = Double.NaN;
    private boolean homing;
    private boolean homed;

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

    public double getStatorCurrentAmps() {
        return pitchMotor.getStatorCurrent().getValueAsDouble();
    }

    public boolean isHoming() {
        return homing;
    }

    public boolean isHomed() {
        return homed;
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
        homingStartTimestampSeconds = Timer.getFPGATimestamp();
        highCurrentStartTimestampSeconds = Double.NaN;
    }

    public void updateControl() {
        if (DriverStation.isDisabled()) {
            homing = false;
            setMechanismOutput(0.0);
        } else if (homing) {
            updateHomingControl();
        } else if (homed) {
            updateMotionProfile();
            updatePitchController();
        } else {
            setMechanismOutput(0.0);
            targetPitchDegrees = clampPitchToTravelWindow(getPitchDegrees());
            profiledPitchDegrees = targetPitchDegrees;
            profiledPitchVelocityDegreesPerSecond = 0.0;
            lastProfileUpdateTimeSeconds = Timer.getFPGATimestamp();
        }
    }

    private void updateHomingControl() {
        double nowSeconds = Timer.getFPGATimestamp();
        setMechanismOutput(-TurretPitchConstants.kPitchHomingMotorOutput, false);

        if (nowSeconds - homingStartTimestampSeconds < TurretPitchConstants.kPitchHomingMinRunTimeSeconds) {
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

    private void setTargetPitchDegrees(double requestedPitchDegrees) {
        targetPitchDegrees = clampPitchToTravelWindow(requestedPitchDegrees);
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
}
