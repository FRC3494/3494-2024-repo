package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    CANSparkMax armMotor;
    double manualPower = 0;
    public Arm(){
        armMotor= new CANSparkMax(Constants.Arm.armMotor, MotorType.kBrushless);
        armMotor.setIdleMode(IdleMode.kBrake);

        armMotor.getPIDController().setOutputRange(-1, 1);
        // armMotor.getPIDController().setFeedbackDevice(armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle));
        armMotor.getPIDController().setP(0.01);
    }
    public void setTargetAngle(double ticks, double arbFFVoltage) {
        // armMotor.getPIDController().setReference(ticks,
        //         ControlType.kPosition);
        armMotor.getPIDController().setReference(ticks, CANSparkMax.ControlType.kPosition, 0, arbFFVoltage,
                SparkPIDController.ArbFFUnits.kVoltage);
    }
    public double getCurrentAngle(){
        return armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
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
        return armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
    }
}
