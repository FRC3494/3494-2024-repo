package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    CANSparkMax armMotor;
    public Arm(){
        armMotor= new CANSparkMax(Constants.Arm.armMotor, MotorType.kBrushless);
        armMotor.setIdleMode(IdleMode.kBrake);

        armMotor.getPIDController().setOutputRange(-1, 1);
        armMotor.getPIDController().setFeedbackDevice(armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle));
    }
    public void setTargetAngle(double ticks) {
        armMotor.getPIDController().setReference(ticks,
                ControlType.kPosition);
    }
    public double getCurrentAngle(){
        return armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
    }
    public void setMotorPower(double speed){
        armMotor.set(speed);
    }
}
