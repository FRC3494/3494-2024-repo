package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;
import frc.robot.Main;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    public CANSparkFlex mainMotor;
    public CANSparkFlex followerMotor;
    public Elevator(){
        mainMotor = new CANSparkFlex(Constants.Elevator.mainMotor, MotorType.kBrushless);
        followerMotor = new CANSparkFlex(Constants.Elevator.followerMotor, MotorType.kBrushless);
        
        mainMotor.setIdleMode(IdleMode.kBrake);
        followerMotor.setIdleMode(IdleMode.kBrake);

        mainMotor.getPIDController().setOutputRange(-1, 1);
        followerMotor.getPIDController().setOutputRange(-1, 1);

        followerMotor.follow(mainMotor);
    }
    public void setElevatorPower(double power){
        power = Math.max(Math.min(power, 1), -1);
        mainMotor.set(power);
    }
    public void setElevatorVoltage(double voltage) {
        mainMotor.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
    }

    public void setElevatorPosition(double position, double arbFFVoltage) {
        mainMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0, arbFFVoltage,
                SparkPIDController.ArbFFUnits.kVoltage);
    }
    public void resetPosition(double position) {
        mainMotor.getEncoder().setPosition(position);
    }
}
