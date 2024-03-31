package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.TeleopRumble;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;
    private double manualPower = 0;
    private ColorSensorV3 leftIntakeColorSensor;
    private ArrayList<Double> currents = new ArrayList<Double>();

    private boolean hadNote = false;
    private boolean hasNoteNow = false;

    private boolean stopIntake = false;
    public boolean inIntake = false;

    public Intake(EventLoop eventloop) {
        intakeMotor = new CANSparkMax(Constants.Intake.mainMotor, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        leftIntakeColorSensor = new ColorSensorV3(Constants.Intake.colorSensorID);
    }

    public double currentAverage(double currentCurrent) {
        currents.add(0, currentCurrent);

        if (currents.size() >= 13) {
            currents.remove(currents.size() - 1);
        }

        double average = 0;

        for (int i = 0; i < currents.size(); i++) {
            average += (double) currents.get(i);
        }

        average /= 13;// currents.size();

        return average;
    }

    public void setMotorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);

        manualPower = power;
        intakeMotor.set(manualPower);
    }

    @Override
    public void periodic() {
        hasNoteNow = (currentAverage(intakeMotor.getOutputCurrent()) > 12.5);// (distOnboard.getRange()<= 8.0 &&
                                                                             // distOnboard.getRange() != -1);
        if (hasNoteNow && !hadNote && inIntake) {
            stopIntake = true;
            (new TeleopRumble(OI.getPrimaryController(), 0.5)).schedule();
            (new TeleopRumble(OI.getDiannaRumbler(), 0.5)).schedule();
        }

        hadNote = hasNoteNow;

        Logger.recordOutput("Intake/Power", intakeMotor.get());
        Logger.recordOutput("Intake/HasNote", hasNoteNow);
        Logger.recordOutput("Intake/StopIntake", stopIntake);
    }

    public double getManualMotorPower() {
        return manualPower;
    }

    public double getSensorProximity() {
        // return distOnboard.getRange();
        return intakeMotor.getOutputCurrent();

        // return leftIntakeColorSensor.getProximity();
    }

    public boolean hasNote() {
        return hasNoteNow;
    }

    public boolean justGotNote() {
        return hasNoteNow && !hadNote;
    }

}
