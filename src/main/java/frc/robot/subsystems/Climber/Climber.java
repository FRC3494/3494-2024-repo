package frc.robot.subsystems.Climber;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;
import frc.robot.Main;
import frc.robot.subsystems.Elevator.ElevatorSensorState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    public CANSparkMax mainMotor;
    public double manualPower = 0 ;

    private DigitalInput bottomMagnetSensor;
    public Climber(){
        mainMotor = new CANSparkMax(Constants.Climber.mainMotor, MotorType.kBrushless);
        
        mainMotor.setIdleMode(IdleMode.kBrake);

        mainMotor.getPIDController().setOutputRange(-1, 1);
        mainMotor.getPIDController().setP(0.1);
        bottomMagnetSensor = new DigitalInput(Constants.Climber.bottomMagnetSensorDIO);
    }
    public void setElevatorPower(double power){
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        mainMotor.set(manualPower);
    }
        @Override
    public void periodic(){
        if(getClimberSensorState() == ElevatorSensorState.BOTTOM){
            resetPosition(0);
        }
    }
    public ElevatorSensorState getClimberSensorState() {
        if (!bottomMagnetSensor.get())
            return ElevatorSensorState.BOTTOM;
        return ElevatorSensorState.MIDDLE;
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
    public double getTicks(){
        return mainMotor.getEncoder().getPosition();
    }
    public double getMotorPower(){
        return mainMotor.getAppliedOutput();
    }
    public double getManualMotorPower(){
        return manualPower;
    }
}
