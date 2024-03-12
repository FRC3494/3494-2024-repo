package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.TeleopRumble;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;


public class Intake extends SubsystemBase {
    CANSparkMax intakeMotor;
    double manualPower = 0;
    ColorSensorV3 leftIntakeColorSensor;
    Rev2mDistanceSensor distOnboard;
    
    boolean hadNote = false;
    boolean hasNoteNow = false;

    boolean stopIntake = false;
    private EventLoop eventLoop;

    public Intake(EventLoop eventloop) {
        distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        distOnboard.setAutomaticMode(true);
        this.eventLoop = eventloop;
        intakeMotor = new CANSparkMax(Constants.Intake.mainMotor, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        leftIntakeColorSensor = new ColorSensorV3(Constants.Intake.colorSensorID);
    }

    public void setMotorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        // if (Math.abs(power) <= 0.1) {
        //     stopIntake = false;
        // }
        // if(stopIntake){
        //     intakeMotor.set(0);
        // }
        // else{
            manualPower = power;
            intakeMotor.set(manualPower);
        // }


    }

    @Override
    public void periodic() {
        hasNoteNow = (distOnboard.getRange()<= 8.0 && distOnboard.getRange() != -1);
        if(hasNoteNow && !hadNote){
            stopIntake = true;
            // (new TeleopRumble(OI.getPrimaryController())).schedule();
        }
        hadNote = hasNoteNow;

    }

    public double getManualMotorPower() {
        return manualPower;
    }

    public double getSensorProximity() {
        return distOnboard.getRange();
        // return leftIntakeColorSensor.getProximity();
    }

    public boolean JustgotNote() {
        hasNoteNow = (distOnboard.getRange()<= 9.0 && distOnboard.getRange() != -1);
        if(hasNoteNow && !hadNote){
            return true;
        }
        return false;

    }
    public boolean hasNote(){
        return hasNoteNow;
    }
    public boolean justGotNote(){
        return hasNoteNow && !hadNote;
    }

}
