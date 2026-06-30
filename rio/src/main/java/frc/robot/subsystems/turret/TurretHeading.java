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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TurretConstants;

public class TurretHeading {
    private final TalonFX headingMotor = new TalonFX(
        TurretConstants.kHeadingMotorCanId,
        new CANBus(TurretConstants.kHeadingMotorCanBus)
    );

    private final DutyCycleEncoder headingEncoder = new DutyCycleEncoder(
        TurretConstants.kThroughBoreEncoderDioChannel,
        1.0,
        0.0
    );

    private double previousRawEncoderRotations;
    private double unwrappedEncoderRotationsFromForward;
    private double targetHeadingDegrees;
    private double profiledHeadingDegrees;
    private double profiledHeadingVelocityDegreesPerSecond;
    private double lastProfileUpdateTimeSeconds;
    private double startupTimestampSeconds;
    private double motorEncoderFallbackHeadingOffsetDegrees;
    private double appliedMotorOutput;
    private boolean encoderWasPresentAtStartup;
    private boolean startupEncoderCheckComplete;
    private boolean usingMotorEncoderFallback;
    private boolean lastEncoderConnected;
    private boolean reportedStartupEncoderFault;
    private boolean reportedFallbackWarning;

    public TurretHeading() {
        headingMotor.getConfigurator().apply(
            new TalonFXConfiguration().withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(TurretConstants.kTurretHeadingSupplyCurrentLimitAmps)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(TurretConstants.kTurretHeadingStatorCurrentLimitAmps)
                    .withStatorCurrentLimitEnable(true)
            )
        );
        headingMotor.setNeutralMode(NeutralModeValue.Brake);

        headingEncoder.setDutyCycleRange(
            TurretConstants.kThroughBoreMinDutyCycle,
            TurretConstants.kThroughBoreMaxDutyCycle
        );

        startupTimestampSeconds = Timer.getFPGATimestamp();
        lastProfileUpdateTimeSeconds = Timer.getFPGATimestamp();

        if (isEncoderConnected()) {
            initializeHeadingFromThroughBore();
            encoderWasPresentAtStartup = true;
            startupEncoderCheckComplete = true;
            lastEncoderConnected = true;
        } else {
            holdCurrentHeading();
        }
    }

    public boolean isEncoderConnected() {
        return headingEncoder.isConnected();
    }

    public boolean wasEncoderPresentAtStartup() {
        return encoderWasPresentAtStartup;
    }

    public boolean isUsingMotorEncoderFallback() {
        return usingMotorEncoderFallback;
    }

    public boolean isHeadingMotionAllowed() {
        return encoderWasPresentAtStartup;
    }

    public String getStatus() {
        if (!startupEncoderCheckComplete) {
            return "WAITING_FOR_THROUGH_BORE";
        }

        if (!encoderWasPresentAtStartup) {
            return "FAULT_THROUGH_BORE_MISSING_AT_STARTUP";
        }

        if (usingMotorEncoderFallback) {
            return "FALLBACK_KRAKEN_ENCODER";
        }

        return "OK_THROUGH_BORE";
    }

    public double getHeadingDegrees() {
        if (encoderWasPresentAtStartup && !usingMotorEncoderFallback) {
            return getThroughBoreHeadingDegrees();
        }

        return getMotorEncoderHeadingDegrees();
    }

    public double getMotorEncoderHeadingDegrees() {
        return motorEncoderFallbackHeadingOffsetDegrees + getRawMotorEncoderHeadingDegrees();
    }

    public double getThroughBoreHeadingDegrees() {
        return TurretConstants.kTurretHeadingSign
            * unwrappedEncoderRotationsFromForward
            * (TurretConstants.kEncoderGearTeeth / TurretConstants.kTurretGearTeeth)
            * 360.0;
    }

    public double getTargetHeadingDegrees() {
        return targetHeadingDegrees;
    }

    public double getProfiledHeadingDegrees() {
        return profiledHeadingDegrees;
    }

    public double getHeadingErrorDegrees() {
        return profiledHeadingDegrees - getHeadingDegrees();
    }

    public double getAppliedMotorOutput() {
        return appliedMotorOutput;
    }

    public void stepTargetLeft() {
        setTargetHeadingDegrees(targetHeadingDegrees + TurretConstants.kTargetHeadingStepDegrees);
    }

    public void stepTargetRight() {
        setTargetHeadingDegrees(targetHeadingDegrees - TurretConstants.kTargetHeadingStepDegrees);
    }

    public void updateControl() {
        updateHeadingMeasurement();
        updateStartupEncoderCheck();

        if (!isHeadingMotionAllowed()) {
            holdCurrentHeading();
            setMotorOutput(0.0);
            return;
        }

        updateMotionProfile();
        updateHeadingController();
    }

    private double getRawEncoderRotations() {
        return headingEncoder.get();
    }

    private double getForwardEncoderOffsetRotations() {
        return TurretConstants.kForwardEncoderOffsetDegrees / 360.0;
    }

    private double getRawMotorEncoderHeadingDegrees() {
        return TurretConstants.kHeadingMotorPositionSign
            * headingMotor.getPosition().getValueAsDouble()
            / TurretConstants.kMotorToTurretReduction
            * 360.0;
    }

    private void initializeHeadingFromThroughBore() {
        previousRawEncoderRotations = getRawEncoderRotations();
        unwrappedEncoderRotationsFromForward = MathUtil.inputModulus(
            previousRawEncoderRotations - getForwardEncoderOffsetRotations(),
            -0.5,
            0.5
        );
        usingMotorEncoderFallback = false;
        syncMotorEncoderFallbackOffset();
        holdCurrentHeading();
    }

    private void updateHeadingMeasurement() {
        boolean encoderConnected = isEncoderConnected();

        if (encoderConnected && !startupEncoderCheckComplete) {
            initializeHeadingFromThroughBore();
            encoderWasPresentAtStartup = true;
            startupEncoderCheckComplete = true;
            lastEncoderConnected = true;
            return;
        }

        if (!encoderWasPresentAtStartup) {
            lastEncoderConnected = encoderConnected;
            return;
        }

        if (!encoderConnected) {
            usingMotorEncoderFallback = true;
            reportFallbackWarningOnce();
            lastEncoderConnected = false;
            return;
        }

        if (!lastEncoderConnected) {
            alignThroughBoreUnwrapToMotorFallback();
        } else {
            updateUnwrappedEncoderPosition();
        }

        usingMotorEncoderFallback = false;
        syncMotorEncoderFallbackOffset();
        lastEncoderConnected = true;
    }

    private void updateUnwrappedEncoderPosition() {
        double rawEncoderRotations = getRawEncoderRotations();
        double deltaRotations = MathUtil.inputModulus(
            rawEncoderRotations - previousRawEncoderRotations,
            -0.5,
            0.5
        );

        unwrappedEncoderRotationsFromForward += deltaRotations;
        previousRawEncoderRotations = rawEncoderRotations;
    }

    private void alignThroughBoreUnwrapToMotorFallback() {
        unwrappedEncoderRotationsFromForward = getMotorEncoderHeadingDegrees()
            / (TurretConstants.kTurretHeadingSign * 360.0)
            * (TurretConstants.kTurretGearTeeth / TurretConstants.kEncoderGearTeeth);
        previousRawEncoderRotations = getRawEncoderRotations();
    }

    private void syncMotorEncoderFallbackOffset() {
        motorEncoderFallbackHeadingOffsetDegrees = getThroughBoreHeadingDegrees()
            - getRawMotorEncoderHeadingDegrees();
    }

    private void updateStartupEncoderCheck() {
        if (startupEncoderCheckComplete) {
            return;
        }

        if (Timer.getFPGATimestamp() - startupTimestampSeconds
            < TurretConstants.kHeadingEncoderStartupGraceSeconds) {
            return;
        }

        startupEncoderCheckComplete = true;
        reportStartupEncoderFaultOnce();
    }

    private void holdCurrentHeading() {
        targetHeadingDegrees = clampHeadingToTravelWindow(getHeadingDegrees());
        profiledHeadingDegrees = targetHeadingDegrees;
        profiledHeadingVelocityDegreesPerSecond = 0.0;
        lastProfileUpdateTimeSeconds = Timer.getFPGATimestamp();
    }

    public void setTargetHeadingDegrees(double requestedHeadingDegrees) {
        if (!isHeadingMotionAllowed()) {
            holdCurrentHeading();
            return;
        }

        targetHeadingDegrees = wrapHeadingIntoTravelWindow(requestedHeadingDegrees);
    }

    private double wrapHeadingIntoTravelWindow(double headingDegrees) {
        double wrappedHeadingDegrees = headingDegrees;

        while (wrappedHeadingDegrees > TurretConstants.kMaxTurretHeadingDegrees) {
            wrappedHeadingDegrees -= TurretConstants.kTurretHeadingWrapDegrees;
        }

        while (wrappedHeadingDegrees < TurretConstants.kMinTurretHeadingDegrees) {
            wrappedHeadingDegrees += TurretConstants.kTurretHeadingWrapDegrees;
        }

        return clampHeadingToTravelWindow(wrappedHeadingDegrees);
    }

    private double clampHeadingToTravelWindow(double headingDegrees) {
        return MathUtil.clamp(
            headingDegrees,
            TurretConstants.kMinTurretHeadingDegrees,
            TurretConstants.kMaxTurretHeadingDegrees
        );
    }

    private void updateMotionProfile() {
        double nowSeconds = Timer.getFPGATimestamp();
        double dtSeconds = MathUtil.clamp(nowSeconds - lastProfileUpdateTimeSeconds, 0.0, 0.05);
        lastProfileUpdateTimeSeconds = nowSeconds;

        double profileErrorDegrees = targetHeadingDegrees - profiledHeadingDegrees;
        if (dtSeconds <= 0.0) {
            return;
        }

        if (Math.abs(profileErrorDegrees) <= TurretConstants.kHeadingToleranceDegrees
            && Math.abs(profiledHeadingVelocityDegreesPerSecond) <= 1.0) {
            profiledHeadingDegrees = targetHeadingDegrees;
            profiledHeadingVelocityDegreesPerSecond = 0.0;
            return;
        }

        double desiredVelocityMagnitude = Math.min(
            TurretConstants.kMaxTurretHeadingVelocityDegreesPerSecond,
            Math.sqrt(
                2.0
                    * TurretConstants.kMaxTurretHeadingAccelerationDegreesPerSecondSquared
                    * Math.abs(profileErrorDegrees)
            )
        );
        double desiredVelocityDegreesPerSecond = Math.copySign(
            desiredVelocityMagnitude,
            profileErrorDegrees
        );

        double maxVelocityStep = TurretConstants.kMaxTurretHeadingAccelerationDegreesPerSecondSquared
            * dtSeconds;
        profiledHeadingVelocityDegreesPerSecond = MathUtil.clamp(
            desiredVelocityDegreesPerSecond,
            profiledHeadingVelocityDegreesPerSecond - maxVelocityStep,
            profiledHeadingVelocityDegreesPerSecond + maxVelocityStep
        );

        double nextProfiledHeadingDegrees = profiledHeadingDegrees
            + profiledHeadingVelocityDegreesPerSecond * dtSeconds;

        if (Math.signum(targetHeadingDegrees - nextProfiledHeadingDegrees)
            != Math.signum(profileErrorDegrees)) {
            profiledHeadingDegrees = targetHeadingDegrees;
            profiledHeadingVelocityDegreesPerSecond = 0.0;
            return;
        }

        profiledHeadingDegrees = clampHeadingToTravelWindow(nextProfiledHeadingDegrees);
    }

    private void updateHeadingController() {
        if (DriverStation.isDisabled()) {
            profiledHeadingVelocityDegreesPerSecond = 0.0;
            setMotorOutput(0.0);
            return;
        }

        double headingErrorDegrees = getHeadingErrorDegrees();
        if (Math.abs(headingErrorDegrees) <= TurretConstants.kHeadingToleranceDegrees) {
            setMotorOutput(0.0);
            return;
        }

        double motorOutput = TurretConstants.kHeadingMotorOutputSign
            * (
                TurretConstants.kTurretHeadingKv * profiledHeadingVelocityDegreesPerSecond
                    + TurretConstants.kTurretHeadingKp * headingErrorDegrees
            );

        setMotorOutput(
            MathUtil.clamp(
                motorOutput,
                -TurretConstants.kMaxTurretMotorOutput,
                TurretConstants.kMaxTurretMotorOutput
            )
        );
    }

    private void setMotorOutput(double motorOutput) {
        double safeMotorOutput = applyTravelLimits(motorOutput);

        appliedMotorOutput = safeMotorOutput;
        headingMotor.set(safeMotorOutput);
    }

    private double applyTravelLimits(double motorOutput) {
        double currentHeadingDegrees = getHeadingDegrees();

        if (currentHeadingDegrees >= TurretConstants.kMaxTurretHeadingDegrees
            && motorOutput * TurretConstants.kHeadingMotorOutputSign > 0.0) {
            return 0.0;
        }

        if (currentHeadingDegrees <= TurretConstants.kMinTurretHeadingDegrees
            && motorOutput * TurretConstants.kHeadingMotorOutputSign < 0.0) {
            return 0.0;
        }

        return motorOutput;
    }

    private void reportStartupEncoderFaultOnce() {
        if (reportedStartupEncoderFault) {
            return;
        }

        DriverStation.reportError(
            "Turret heading through-bore encoder was missing at startup; heading motion is disabled.",
            false
        );
        reportedStartupEncoderFault = true;
    }

    private void reportFallbackWarningOnce() {
        if (reportedFallbackWarning) {
            return;
        }

        DriverStation.reportWarning(
            "Turret heading through-bore encoder disconnected; falling back to Kraken internal encoder.",
            false
        );
        reportedFallbackWarning = true;
    }
}
