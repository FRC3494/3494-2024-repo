package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    CANSparkMax intakeMotor;
    double manualPower = 0;
    ColorSensorV3 leftIntakeColorSensor;

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.Intake.mainMotor, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        leftIntakeColorSensor = new ColorSensorV3(Constants.Intake.colorSensorID);
    }

    public void setMotorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        intakeMotor.set(manualPower);

    }

    @Override
    public void periodic() {
        System.out.println(leftIntakeColorSensor.getProximity());
    }

    public double getManualMotorPower() {
        return manualPower;
    }

}
