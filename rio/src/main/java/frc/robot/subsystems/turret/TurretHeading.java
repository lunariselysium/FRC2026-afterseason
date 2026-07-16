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
    private double previousRawMotorEncoderHeadingDegrees;
    private double unwrappedEncoderRotationsFromForward;
    private double targetHeadingDegrees;
    private double profiledHeadingDegrees;
    private double profiledHeadingVelocityDegreesPerSecond;
    private double lastProfileUpdateTimeSeconds;
    private double startupTimestampSeconds;
    private double motorEncoderFallbackHeadingOffsetDegrees;
    private double appliedMotorOutput;
    private double appliedSysIdMechanismVoltage;
    private double encoderMotorGuidanceErrorDegrees;
    private int motorEncoderGuidanceCorrectionCount;
    private boolean encoderWasPresentAtStartup;
    private boolean startupEncoderCheckComplete;
    private boolean usingMotorEncoderFallback;
    private boolean lastEncoderConnected;
    private boolean sysIdActive;
    private boolean usedMotorEncoderGuidance;
    private boolean reportedStartupEncoderFault;
    private boolean reportedFallbackWarning;
    private boolean reportedSysIdNotAllowedWarning;

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
        previousRawMotorEncoderHeadingDegrees = getRawMotorEncoderHeadingDegrees();

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
        return unwrappedEncoderRotationsFromForward * getHeadingDegreesPerEncoderRotation();
    }

    public double getRawThroughBoreEncoderRotations() {
        return getRawEncoderRotations();
    }

    public double getRawThroughBoreEncoderDegrees() {
        return TurretHeadingMath.encoderRotationsToDegrees(
            getRawThroughBoreEncoderRotations()
        );
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

    public double getHeadingVelocityDegreesPerSecond() {
        return TurretConstants.kHeadingMotorPositionSign
            * headingMotor.getVelocity().getValueAsDouble()
            / TurretConstants.kMotorToTurretReduction
            * 360.0;
    }

    public double getAppliedSysIdMechanismVoltage() {
        return appliedSysIdMechanismVoltage;
    }

    public double getEncoderMotorGuidanceErrorDegrees() {
        return encoderMotorGuidanceErrorDegrees;
    }

    public boolean didUseMotorEncoderGuidance() {
        return usedMotorEncoderGuidance;
    }

    public int getMotorEncoderGuidanceCorrectionCount() {
        return motorEncoderGuidanceCorrectionCount;
    }

    public double getMeasuredMechanismVoltage() {
        return TurretConstants.kHeadingMotorOutputSign
            * headingMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getStatorCurrentAmps() {
        return headingMotor.getStatorCurrent().getValueAsDouble();
    }

    public boolean isSysIdActive() {
        return sysIdActive;
    }

    public boolean isAtTarget() {
        return isHeadingMotionAllowed()
            && TurretHeadingMath.isWithinTolerance(
                targetHeadingDegrees,
                getHeadingDegrees(),
                TurretConstants.kHeadingReadyToleranceDegrees
            );
    }

    public void stepTargetLeft() {
        setTargetHeadingDegrees(targetHeadingDegrees + TurretConstants.kTargetHeadingStepDegrees);
    }

    public void stepTargetRight() {
        setTargetHeadingDegrees(targetHeadingDegrees - TurretConstants.kTargetHeadingStepDegrees);
    }

    public void resetEncoderRotationToTargetHeading() {
        if (!encoderWasPresentAtStartup || !isEncoderConnected()) {
            return;
        }

        previousRawEncoderRotations = getRawEncoderRotations();
        previousRawMotorEncoderHeadingDegrees = getRawMotorEncoderHeadingDegrees();
        unwrappedEncoderRotationsFromForward =
            TurretHeadingMath.chooseNearestEquivalentEncoderRotationsFromForward(
                previousRawEncoderRotations - getForwardEncoderOffsetRotations(),
                targetHeadingDegrees,
                getHeadingDegreesPerEncoderRotation()
            );
        usingMotorEncoderFallback = false;
        lastEncoderConnected = true;
        encoderMotorGuidanceErrorDegrees = 0.0;
        usedMotorEncoderGuidance = false;
        syncMotorEncoderFallbackOffset();
    }

    public void shiftEncoderRotationTowardPositiveHeading() {
        shiftEncoderRotationTowardHeadingDirection(1.0);
    }

    public void shiftEncoderRotationTowardNegativeHeading() {
        shiftEncoderRotationTowardHeadingDirection(-1.0);
    }

    public void prepareSysId() {
        holdCurrentHeading();
        sysIdActive = false;
        reportedSysIdNotAllowedWarning = false;
        setMechanismVoltage(0.0);
    }

    public void runSysIdVoltage(double mechanismVoltage) {
        if (Math.abs(mechanismVoltage) < 1.0e-6) {
            stopSysId();
            return;
        }

        if (!isHeadingMotionAllowed()) {
            reportSysIdNotAllowedWarningOnce();
            stopSysId();
            return;
        }

        sysIdActive = true;
        setMechanismVoltage(mechanismVoltage);
    }

    public void stopSysId() {
        sysIdActive = false;
        holdCurrentHeading();
        setMechanismVoltage(0.0);
    }

    public void updateControl() {
        updateHeadingMeasurement();
        updateStartupEncoderCheck();

        if (DriverStation.isDisabled()) {
            sysIdActive = false;
            setMotorOutput(0.0);
            return;
        }

        if (!isHeadingMotionAllowed()) {
            holdCurrentHeading();
            setMotorOutput(0.0);
            return;
        }

        if (sysIdActive) {
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

    private double getHeadingDegreesPerEncoderRotation() {
        return TurretConstants.kTurretHeadingSign
            * (TurretConstants.kEncoderGearTeeth / TurretConstants.kTurretGearTeeth)
            * 360.0;
    }

    private void initializeHeadingFromThroughBore() {
        previousRawEncoderRotations = getRawEncoderRotations();
        previousRawMotorEncoderHeadingDegrees = getRawMotorEncoderHeadingDegrees();
        unwrappedEncoderRotationsFromForward =
            TurretHeadingMath.initializeEncoderRotationsFromKnownHeading(
                previousRawEncoderRotations,
                TurretConstants.kForwardEncoderOffsetDegrees,
                TurretConstants.kStartupTurretHeadingDegrees,
                getHeadingDegreesPerEncoderRotation()
            );
        usingMotorEncoderFallback = false;
        encoderMotorGuidanceErrorDegrees = 0.0;
        usedMotorEncoderGuidance = false;
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
        double rawMotorEncoderHeadingDegrees = getRawMotorEncoderHeadingDegrees();
        double continuityDeltaRotations = MathUtil.inputModulus(
            rawEncoderRotations - previousRawEncoderRotations,
            -0.5,
            0.5
        );
        double continuityEncoderRotationsFromForward =
            unwrappedEncoderRotationsFromForward + continuityDeltaRotations;
        double motorEncoderDeltaDegrees = rawMotorEncoderHeadingDegrees
            - previousRawMotorEncoderHeadingDegrees;
        double motorPredictedHeadingDegrees = getThroughBoreHeadingDegrees()
            + motorEncoderDeltaDegrees;
        TurretHeadingMath.EncoderUnwrapResult unwrapResult =
            TurretHeadingMath.chooseEncoderUnwrap(
                rawEncoderRotations - getForwardEncoderOffsetRotations(),
                continuityEncoderRotationsFromForward,
                motorPredictedHeadingDegrees,
                getHeadingDegreesPerEncoderRotation()
            );

        unwrappedEncoderRotationsFromForward = unwrapResult.encoderRotationsFromForward();
        encoderMotorGuidanceErrorDegrees = unwrapResult.motorGuidanceErrorDegrees();
        usedMotorEncoderGuidance = unwrapResult.usedMotorGuidance();
        if (usedMotorEncoderGuidance) {
            motorEncoderGuidanceCorrectionCount++;
        }
        previousRawEncoderRotations = rawEncoderRotations;
        previousRawMotorEncoderHeadingDegrees = rawMotorEncoderHeadingDegrees;
    }

    private void shiftEncoderRotationTowardHeadingDirection(double headingDirection) {
        if (!encoderWasPresentAtStartup || !isEncoderConnected()) {
            return;
        }

        previousRawEncoderRotations = getRawEncoderRotations();
        previousRawMotorEncoderHeadingDegrees = getRawMotorEncoderHeadingDegrees();
        unwrappedEncoderRotationsFromForward =
            TurretHeadingMath.shiftEncoderRotationTowardHeadingDirection(
                unwrappedEncoderRotationsFromForward,
                headingDirection,
                getHeadingDegreesPerEncoderRotation()
            );
        usingMotorEncoderFallback = false;
        lastEncoderConnected = true;
        encoderMotorGuidanceErrorDegrees = 0.0;
        usedMotorEncoderGuidance = false;
        syncMotorEncoderFallbackOffset();
    }

    private void alignThroughBoreUnwrapToMotorFallback() {
        previousRawEncoderRotations = getRawEncoderRotations();
        double motorFallbackHeadingDegrees = getMotorEncoderHeadingDegrees();
        unwrappedEncoderRotationsFromForward =
            TurretHeadingMath.chooseNearestEquivalentEncoderRotationsFromForward(
                previousRawEncoderRotations - getForwardEncoderOffsetRotations(),
                motorFallbackHeadingDegrees,
                getHeadingDegreesPerEncoderRotation()
            );
        previousRawMotorEncoderHeadingDegrees = getRawMotorEncoderHeadingDegrees();
        encoderMotorGuidanceErrorDegrees = Math.abs(
            getThroughBoreHeadingDegrees() - motorFallbackHeadingDegrees
        );
        usedMotorEncoderGuidance = true;
        motorEncoderGuidanceCorrectionCount++;
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
        return TurretHeadingMath.chooseNearestEquivalentInWindow(
            headingDegrees,
            profiledHeadingDegrees,
            TurretConstants.kMinTurretHeadingDegrees,
            TurretConstants.kMaxTurretHeadingDegrees,
            TurretConstants.kTurretHeadingWrapDegrees
        );
    }

    private double clampHeadingToTravelWindow(double headingDegrees) {
        return TurretHeadingMath.clampHeadingToTravelWindow(
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
        double headingErrorDegrees = getHeadingErrorDegrees();
        if (Math.abs(headingErrorDegrees) <= TurretConstants.kHeadingToleranceDegrees) {
            setMotorOutput(0.0);
            return;
        }

        double staticFeedforwardDirection = Math.signum(profiledHeadingVelocityDegreesPerSecond);
        if (staticFeedforwardDirection == 0.0) {
            staticFeedforwardDirection = Math.signum(headingErrorDegrees);
        }

        double mechanismOutput =
            TurretConstants.kTurretHeadingKs * staticFeedforwardDirection
                + TurretConstants.kTurretHeadingKv * profiledHeadingVelocityDegreesPerSecond
                + TurretConstants.kTurretHeadingKp * headingErrorDegrees;
        double motorOutput = TurretConstants.kHeadingMotorOutputSign * mechanismOutput;

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
        appliedSysIdMechanismVoltage = 0.0;
        headingMotor.set(safeMotorOutput);
    }

    private void setMechanismVoltage(double mechanismVoltage) {
        double safeMechanismVoltage = applyVoltageTravelLimits(mechanismVoltage);

        appliedMotorOutput = 0.0;
        appliedSysIdMechanismVoltage = safeMechanismVoltage;
        headingMotor.setVoltage(
            TurretConstants.kHeadingMotorOutputSign * safeMechanismVoltage
        );
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

    private double applyVoltageTravelLimits(double mechanismVoltage) {
        double currentHeadingDegrees = getHeadingDegrees();

        if (currentHeadingDegrees >= TurretConstants.kMaxTurretHeadingDegrees
            && mechanismVoltage > 0.0) {
            return 0.0;
        }

        if (currentHeadingDegrees <= TurretConstants.kMinTurretHeadingDegrees
            && mechanismVoltage < 0.0) {
            return 0.0;
        }

        return mechanismVoltage;
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

    private void reportSysIdNotAllowedWarningOnce() {
        if (reportedSysIdNotAllowedWarning) {
            return;
        }

        DriverStation.reportWarning(
            "Turret heading SysId requested before heading motion is allowed; heading motor will stay stopped.",
            false
        );
        reportedSysIdNotAllowedWarning = true;
    }
}
