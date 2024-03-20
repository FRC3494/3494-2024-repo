package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    CANSparkMax armMotor;
    double manualPower = 0;
    private double targetPosition;
    public Arm(){
        armMotor= new CANSparkMax(Constants.Arm.armMotor, MotorType.kBrushless);
        armMotor.setIdleMode(IdleMode.kCoast);//was kBrake

        armMotor.getPIDController().setP(2);//1.5
        armMotor.getPIDController().setFF(0.5);
        armMotor.getPIDController().setOutputRange(-0.7, 0.7);
        armMotor.getPIDController().setFeedbackDevice(armMotor.getAlternateEncoder(8192));
        // armMotor.getPIDController().setFeedbackDevice(armMotor.getAbsoluteEncoder(Type.kDutyCycle));
        targetPosition = armMotor.getAlternateEncoder(8192).getPosition();
        // targetPosition = armMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
        
    }
    public void setBrakes(IdleMode neutralMode) {
        this.armMotor.setIdleMode(neutralMode);
    }
    public void setTargetAngle(double ticks, double arbFFVoltage) {
        // armMotor.getPIDController().setReference(ticks,
        //         ControlType.kPosition);
        targetPosition = ticks  + Constants.Arm.globalArmOffset; 
        //if (targetPosition > 0.0){ targetPosition = 0.0;}
        armMotor.getPIDController().setReference(ticks, CANSparkMax.ControlType.kPosition, 0, arbFFVoltage,
                SparkPIDController.ArbFFUnits.kVoltage);
    }
    // public double getCurrentAngle(){
    //     return armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
    // }
    @Override
    public void periodic(){
        if (DriverStation.isEnabled()) this.setBrakes(IdleMode.kBrake);
    }

    public void setMotorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        // System.out.println(manualPower);
        armMotor.set(manualPower);
    }
    public double getManualMotorPower(){
        return manualPower;
    }
    public double getRelativeTicks(){
        return armMotor.getEncoder().getPosition();
    }
    public double getAbsoluteTicks(){
        return armMotor.getAlternateEncoder(8192).getPosition();
        // return armMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }
    public double getTargetPosition(){
        return targetPosition;
    }
}
