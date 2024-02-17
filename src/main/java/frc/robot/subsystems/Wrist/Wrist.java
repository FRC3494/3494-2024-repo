package frc.robot.subsystems.Wrist;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import frc.robot.Constants;
import frc.robot.Main;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    public CANSparkMax wristMotor;
    private double manualPower = 0;
    public Wrist(){
        wristMotor = new CANSparkMax(Constants.Wrist.mainMotor, MotorType.kBrushless);
        
        wristMotor.getPIDController().setP(2);
        wristMotor.getPIDController().setOutputRange(-1, 1);
        wristMotor.getPIDController().setFeedbackDevice(wristMotor.getAlternateEncoder(8192));
        wristMotor.setIdleMode(IdleMode.kBrake);
        // wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        // wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }
    public void setWristPower(double power){
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        wristMotor.set(manualPower);
    }
    public void setWristVoltage(double voltage) {
        wristMotor.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
    }

    public void setWristPosition(double position, double arbFFVoltage) {
        wristMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0, arbFFVoltage,
                SparkPIDController.ArbFFUnits.kVoltage);
    }
    public void resetPosition(double position) {
        wristMotor.getEncoder().setPosition(position);
    }
    public double getManualMotorPower(){
        return manualPower;
    }
    public double getRelativeTicks(){
        return wristMotor.getEncoder().getPosition();
    }
    public double getAbsoluteTicks(){
        return wristMotor.getAlternateEncoder(8192).getPosition();
        // return wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
    }
}
