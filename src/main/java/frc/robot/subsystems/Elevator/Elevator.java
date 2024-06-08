package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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

        mainMotor.setIdleMode(IdleMode.kCoast);// was kBrake

        mainMotor.getPIDController().setOutputRange(-1.0, 1.0);// STATE was 0.75
        mainMotor.getPIDController().setP(0.05);

    }

    public void setElevatorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        mainMotor.set(manualPower);
    }

    public void setElevatorVoltage(double voltage) {
        mainMotor.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
    }

    public void setBrakes(IdleMode neutralMode) {
        this.mainMotor.setIdleMode(neutralMode);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled())
            this.setBrakes(IdleMode.kBrake);

        if (getElevatorSensorState() == ElevatorSensorState.BOTTOM) {
            mainMotor.getEncoder().setPosition(0);
        } else if (getElevatorSensorState() == ElevatorSensorState.TOP) {
            // THE TOP of TH ELEVATOR IN TICKS
            mainMotor.getEncoder().setPosition(-62.3);
        }

        Logger.recordOutput("Elevator/ElevatorSensorState", getElevatorSensorState());
        Logger.recordOutput("Elevator/Power", mainMotor.get());
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

    public double getManualMotorPower() {
        return manualPower;
    }

    public double getTicks() {
        return mainMotor.getEncoder().getPosition();
    }
}
