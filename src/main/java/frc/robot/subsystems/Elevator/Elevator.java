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
    public CANSparkFlex mainMotor;
    public CANSparkFlex followerMotor;

    private DigitalInput bottomMagnetSensor;
    private DigitalInput topMagnetSensor;

    public Elevator() {
        mainMotor = new CANSparkFlex(Constants.Elevator.mainMotor, MotorType.kBrushless);
        followerMotor = new CANSparkFlex(Constants.Elevator.followerMotor, MotorType.kBrushless);

        bottomMagnetSensor = new DigitalInput(Constants.Elevator.bottomMagnetSensorDIO);
        topMagnetSensor = new DigitalInput(Constants.Elevator.topMagnetSensorDIO);

        mainMotor.setIdleMode(IdleMode.kBrake);
        followerMotor.setIdleMode(IdleMode.kBrake);

        mainMotor.getPIDController().setOutputRange(-1, 1);
        followerMotor.getPIDController().setOutputRange(-1, 1);

        followerMotor.follow(mainMotor);
    }

    public void setElevatorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        mainMotor.set(power);
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
        if (bottomMagnetSensor.get())
            return ElevatorSensorState.BOTTOM;

        if (topMagnetSensor.get())
            return ElevatorSensorState.TOP;

        return ElevatorSensorState.MIDDLE;
    }

    public void setElevatorPosition(double position, double arbFFVoltage) {
        mainMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0, arbFFVoltage,
                SparkPIDController.ArbFFUnits.kVoltage);
    }

    public void resetPosition(double position) {
        mainMotor.getEncoder().setPosition(position);
    }
}
