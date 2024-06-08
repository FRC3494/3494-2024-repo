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
    public CANSparkMax mainMotor;
    public double manualPower = 0;

    public RatchetServo ratchetServo = new RatchetServo(Constants.Climber.pwmServoPort);
    public boolean ratchetEngaged = false;

    private DigitalInput bottomMagnetSensor;

    public Climber() {
        mainMotor = new CANSparkMax(Constants.Climber.mainMotor, MotorType.kBrushless);

        mainMotor.setIdleMode(IdleMode.kBrake);

        mainMotor.getPIDController().setOutputRange(-1, 1);
        mainMotor.getPIDController().setP(0.1);
        bottomMagnetSensor = new DigitalInput(Constants.Climber.bottomMagnetSensorDIO);
    }

    public double getCurrentPosition() {
        return mainMotor.getEncoder().getPosition();
    }

    public void disenageRatchet() {
        System.out.println("disengaged: rachet to 0.5");
        ratchetServo.set(0.3);
        ratchetEngaged = false;
    }

    public void engageRatchet() {
        System.out.println("engaged: rachet to 1.0");
        ratchetServo.set(.9);
        ratchetEngaged = true;
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

        Logger.recordOutput("Climber/ClimberPower", manualPower);
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
