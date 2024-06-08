package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    CANSparkMax armMotor;
    double manualPower = 0;
    private double targetPosition;

    public Arm() {
        armMotor = new CANSparkMax(Constants.Arm.armMotor, MotorType.kBrushless);
        armMotor.setIdleMode(IdleMode.kCoast);// was kBrake

        armMotor.getPIDController().setP(2);// 1.5
        armMotor.getPIDController().setFF(0.5);
        armMotor.getPIDController().setOutputRange(-0.7, 0.7);
        armMotor.getPIDController().setFeedbackDevice(armMotor.getAlternateEncoder(8192));

        targetPosition = armMotor.getAlternateEncoder(8192).getPosition();
    }

    public void setBrakes(IdleMode neutralMode) {
        this.armMotor.setIdleMode(neutralMode);
    }

    public void setTargetAngle(double ticks, double arbFFVoltage) {
        targetPosition = ticks + Constants.Arm.globalArmOffset;

        armMotor.getPIDController().setReference(ticks, CANSparkMax.ControlType.kPosition, 0, arbFFVoltage,
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled())
            this.setBrakes(IdleMode.kBrake);

        Logger.recordOutput("Arm/TargetPosition", targetPosition);
        Logger.recordOutput("Arm/RelativeTicks", getRelativeTicks());
        Logger.recordOutput("Arm/AbsoluteTicks", getAbsoluteTicks());
    }

    public void setMotorPower(double power) {
        manualPower = Math.max(Math.min(power, 1), -1);

        armMotor.set(manualPower);
    }

    public double getManualMotorPower() {
        return manualPower;
    }

    public double getRelativeTicks() {
        return armMotor.getEncoder().getPosition();
    }

    public double getAbsoluteTicks() {
        return armMotor.getAlternateEncoder(8192).getPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    }
}
