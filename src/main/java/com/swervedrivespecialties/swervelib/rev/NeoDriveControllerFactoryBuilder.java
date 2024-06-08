package com.swervedrivespecialties.swervelib.rev;

import static com.swervedrivespecialties.swervelib.rev.RevUtils.checkNeoError;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;

public final class NeoDriveControllerFactoryBuilder {
    private double nominalVoltage = Double.NaN;
    private double rampRate = Double.NaN;
    private double currentLimit = Double.NaN;

    public NeoDriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public NeoDriveControllerFactoryBuilder withRampRate(double rampRate) {
        this.rampRate = rampRate;
        return this;
    }

    public boolean hasRampRate() {
        return Double.isFinite(rampRate);
    }

    public NeoDriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer id, ModuleConfiguration moduleConfiguration) {
            CANSparkMax motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
            motor.setInverted(moduleConfiguration.isDriveInverted());

            // Setup voltage compensation
            if (hasVoltageCompensation()) {
                checkNeoError(motor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");
            }

            if (hasRampRate()) {
                checkNeoError(motor.setClosedLoopRampRate(currentLimit), "Failed to set closed loop ramp rate for NEO");
                checkNeoError(motor.setOpenLoopRampRate(currentLimit), "Failed to set open loop ramp rate for NEO");
            }

            if (hasCurrentLimit()) {
                checkNeoError(motor.setSmartCurrentLimit((int) currentLimit), "Failed to set current limit for NEO");
            }

            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100),
                    "Failed to set periodic status frame 0 rate");
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20),
                    "Failed to set periodic status frame 1 rate");
            checkNeoError(motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20),
                    "Failed to set periodic status frame 2 rate");
            // Set neutral mode to brake
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            System.out.println("WHEEEL DIATERE------------------------------------------:"
                    + moduleConfiguration.getWheelDiameter());
            // Setup encoder
            RelativeEncoder encoder = motor.getEncoder();
            double positionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter()
                    * moduleConfiguration.getDriveReduction();
            encoder.setPositionConversionFactor(positionConversionFactor);
            encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);

            // encoder.setAverageDepth(2);
            // encoder.setMeasurementPeriod(16);

            return new ControllerImplementation(motor, encoder);
        }
    }

    private static class ControllerImplementation implements DriveController {
        private final CANSparkMax motor;
        private final RelativeEncoder encoder;

        private ControllerImplementation(CANSparkMax motor, RelativeEncoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setVoltage(voltage);
        }

        @Override
        public double getDistanceTravelled() {
            return encoder.getPosition();

        }

        @Override
        public double getVelocity() {
            return encoder.getVelocity();
        }
    }
}
