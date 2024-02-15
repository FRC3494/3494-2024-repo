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
    public Wrist(){
        wristMotor = new CANSparkMax(Constants.Elevator.mainMotor, MotorType.kBrushless);
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.getPIDController().setOutputRange(-1, 1);
        wristMotor.getPIDController().setFeedbackDevice(wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle));


        wristMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Wrist.MAX_POSITION);
        wristMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Wrist.MIN_POSITION);

        wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }
    public void setWristPower(double power){
        power = Math.max(Math.min(power, 1), -1);
        wristMotor.set(power);
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
}
