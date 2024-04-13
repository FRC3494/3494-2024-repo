package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
    public CANSparkMax wristMotor;
    private double manualPower = 0;
    private double targetPos = 0;
    private boolean targeting = false;
    // private DigitalInput bottomMagnetSensor;
    public Wrist(){
        wristMotor = new CANSparkMax(Constants.Wrist.mainMotor, MotorType.kBrushless);
        
        wristMotor.getPIDController().setP(0.001);
        // wristMotor.getPIDController().setD(0.0);
        wristMotor.getPIDController().setFF(0.0);
        wristMotor.getPIDController().setOutputRange(-0.1, 0.1);
        wristMotor.getPIDController().setFeedbackDevice(wristMotor.getAlternateEncoder(8192));
        
        wristMotor.setIdleMode(IdleMode.kBrake);

        // bottomMagnetSensor = new DigitalInput(Constants.Climber.bottomMagnetSensorDIO);
        // wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        // wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    @Override
    public void periodic(){
        double error = targetPos - wristMotor.getAlternateEncoder(8192).getPosition();
        error *= 100;
        double kP = 0.04;

        if(targeting){
            double PIDpower = error*kP;
            PIDpower = Math.min(0.2, PIDpower);
            PIDpower = Math.max(PIDpower, -0.2);

            // System.out.println(PIDpower);
            wristMotor.set(PIDpower);
        }

        Logger.recordOutput("Wrist/Power", wristMotor.get());
    }
    public void setWristPower(double power){
        targeting = false;
        power = Math.max(Math.min(power, 1), -1);
        manualPower = power;
        wristMotor.set(manualPower);
    }
    public void setWristVoltage(double voltage) {
        wristMotor.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
    }

    public void setWristPosition(double position, double arbFFVoltage) {
        targetPos = position+Constants.Wrist.globalWristOffset;
        targeting = true;
        // wristMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0, arbFFVoltage,
                // SparkPIDController.ArbFFUnits.kVoltage);
    }
    public void resetPosition(double position) {
        wristMotor.getEncoder().setPosition(position);
    }
    public double getManualMotorPower(){
        return manualPower;
    }
    public double getRelativeTicks(){
        return wristMotor.getEncoder().getPosition();
    }
    public double getAbsoluteTicks(){
        return wristMotor.getAlternateEncoder(8192).getPosition();
        // return wristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
    }
}
