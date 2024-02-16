package frc.robot.subsystems.Elevator;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    public CANSparkMax mainMotor;

    private DigitalInput bottomMagnetSensor;
    private DigitalInput topMagnetSensor;

    public double manualPower = 0;
    public Elevator() {
        mainMotor = new CANSparkMax(Constants.Elevator.mainMotor, MotorType.kBrushless);

        bottomMagnetSensor = new DigitalInput(Constants.Elevator.bottomMagnetSensorDIO);
        topMagnetSensor = new DigitalInput(Constants.Elevator.topMagnetSensorDIO);

        mainMotor.setIdleMode(IdleMode.kBrake);

        mainMotor.getPIDController().setOutputRange(-1, 1);
        mainMotor.getPIDController().setP(0.1);

    }

    public void setElevatorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        mainMotor.set(manualPower);
    }

    public void setElevatorVoltage(double voltage) {
        mainMotor.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
    }

    /**
     * Combines the two Magnet Sensor inputs to generate an enum that can be used
     * for software limiting
     * 
     * @return {@link ElevatorSensorState} currentState
     */
    public ElevatorSensorState getElevatorSensorState() {
        if (!topMagnetSensor.get())
            return ElevatorSensorState.TOP;
        if (!bottomMagnetSensor.get())
            return ElevatorSensorState.BOTTOM;
        return ElevatorSensorState.MIDDLE;
    }

    public void setElevatorPosition(double position, double arbFFVoltage) {
        mainMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0, arbFFVoltage,
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    public void resetPosition(double position) {
        mainMotor.getEncoder().setPosition(position);
    }
    public double getManualMotorPower(){
        return manualPower;
    }
    public double getTicks(){
        return mainMotor.getEncoder().getPosition();
    }
}