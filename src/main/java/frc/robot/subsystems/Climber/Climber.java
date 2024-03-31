package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorSensorState;

public class Climber extends SubsystemBase {
    private CANSparkMax mainMotor;
    private RatchetServo ratchetServo = new RatchetServo(Constants.Climber.pwmServoPort);
    private double manualPower = 0;
    private boolean ratchetEngaged = false;

    private DigitalInput bottomMagnetSensor;

    public Climber() {
        ratchetServo.set(0.0);

        ratchetEngaged = false;
        mainMotor = new CANSparkMax(Constants.Climber.mainMotor, MotorType.kBrushless);

        mainMotor.setIdleMode(IdleMode.kBrake);

        mainMotor.getPIDController().setOutputRange(-1, 1);
        mainMotor.getPIDController().setP(0.1);
        bottomMagnetSensor = new DigitalInput(Constants.Climber.bottomMagnetSensorDIO);
    }

    public void engageRatchet() {
        ratchetServo.set(0.5);
        ratchetEngaged = true;
    }

    public void disengageRatchet() {
        ratchetServo.set(0.0);

        double currentPosition = mainMotor.getEncoder().getPosition();
        mainMotor.getPIDController().setReference(currentPosition - 2.0, CANSparkMax.ControlType.kPosition, 0);
    }

    public void setElevatorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        mainMotor.set(manualPower);
    }

    @Override
    public void periodic() {
        if (getClimberSensorState() == ElevatorSensorState.BOTTOM) {
            resetPosition(0);
        }

        Logger.recordOutput("Climber/RatchetEngaged", ratchetEngaged);
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

    public double getTicks() {
        return mainMotor.getEncoder().getPosition();
    }

    public double getMotorPower() {
        return mainMotor.getAppliedOutput();
    }

    public double getManualMotorPower() {
        return manualPower;
    }

}
