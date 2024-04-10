package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.TeleopRumble;

import java.util.ArrayList;

// import com.revrobotics.*;
// import com.revrobotics.Rev2mDistanceSensor.Port;


public class Intake extends SubsystemBase {
    private boolean currentSensing = true;
    CANSparkMax intakeMotor;
    double manualPower = 0;
    ColorSensorV3 leftIntakeColorSensor;
    // public Rev2mDistanceSensor distOnboard;
    ArrayList currents =  new ArrayList<>();
    
    boolean hadNote = false;
    boolean hasNoteNow = false;

    boolean stopIntake = false;
    EventLoop eventLoop;
    public boolean inIntake = false;
     
    public Intake(EventLoop eventloop) {
        // distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        // distOnboard.setAutomaticMode(true);
        this.eventLoop = eventloop;
        intakeMotor = new CANSparkMax(Constants.Intake.mainMotor, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        leftIntakeColorSensor = new ColorSensorV3(Constants.Intake.colorSensorID);
    }

    public double currentAverage(double currentCurrent){
        currents.add(0, currentCurrent);

		if (currents.size() >= 13){
			currents.remove(currents.size() - 1);
        }
        double average = 0;
        for(int i = 0; i<currents.size(); i++){
            average+= (double)currents.get(i);
        }
        average /= 13;//currents.size();
        return average;

    }
    public boolean isSensing(){
        return currentSensing;
    }
    public void setMotorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        if (Math.abs(power) <= 0.1) {
            stopIntake = false;
        }
        if(stopIntake){
            intakeMotor.set(0);
        }
        else{
            manualPower = power;
            intakeMotor.set(manualPower);
        }


    }

    @Override
    public void periodic() {
        // if(currentSensing){
            hasNoteNow = (currentAverage(intakeMotor.getOutputCurrent())>15);//(distOnboard.getRange()<= 8.0 && distOnboard.getRange() != -1);
            if(hasNoteNow && !hadNote &&  inIntake && !DriverStation.isAutonomous() && currentSensing){
                stopIntake = true;
                (new TeleopRumble(OI.getPrimaryController(), 0.5)).schedule();
                (new TeleopRumble(OI.getDiannaRumbler(), 0.5)).schedule();
            }
            hadNote = hasNoteNow;
        // }
    }

    public void toggleCurrentSensing(){
        System.out.println("Current sensing switched to " + (!currentSensing ? "enabled" : "disable"));
        currentSensing = !currentSensing;
    }


    public double getManualMotorPower() {
        return manualPower;
    }

    public double getSensorProximity() {
        // return distOnboard.getRange();
        return intakeMotor.getOutputCurrent();


        // return leftIntakeColorSensor.getProximity();
    }

    // public boolean JustgotNote() {
    //     hasNoteNow = (distOnboard.getRange()<= 9.0 && distOnboard.getRange() != -1);
    //     if(hasNoteNow && !hadNote){
    //         return true;
    //     }
    //     return false;

    // }
    public boolean hasNote(){
        return hasNoteNow;
    }
    public boolean justGotNote(){
        return hasNoteNow && !hadNote;
    }


}
