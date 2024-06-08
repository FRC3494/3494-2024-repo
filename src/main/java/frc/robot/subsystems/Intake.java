package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.TeleopRumble;

public class Intake extends SubsystemBase {
    private boolean currentSensing = true;
    private CANSparkMax intakeMotor;
    private double manualPower = 0;
    private ArrayList<Double> currents = new ArrayList<Double>();

    private boolean hadNote = false;
    private boolean hasNoteNow = false;

    private boolean stopIntake = false;
    public boolean inIntake = false;

    public Intake(EventLoop eventloop) {
        intakeMotor = new CANSparkMax(Constants.Intake.mainMotor, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);
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

    public boolean isSensing() {
        return currentSensing;
    }

    public void setMotorPower(double power) {
        power = Math.max(Math.min(power, 1), -1);
        if (Math.abs(power) <= 0.1) {
            stopIntake = false;
        }
        if (stopIntake) {
            intakeMotor.set(0);
        } else {
            manualPower = power;
            intakeMotor.set(manualPower);
        }
    }

    @Override
    public void periodic() {
        hasNoteNow = (currentAverage(intakeMotor.getOutputCurrent()) > 15);
        if (hasNoteNow && !hadNote && inIntake && !DriverStation.isAutonomous() && currentSensing) {
            stopIntake = true;
            (new TeleopRumble(OI.getPrimaryController(), 0.5)).schedule();
            (new TeleopRumble(OI.getDiannaRumbler(), 0.5)).schedule();
        }
        hadNote = hasNoteNow;

        Logger.recordOutput("Intake/Power", intakeMotor.get());
        Logger.recordOutput("Intake/HasNote", hasNoteNow);
        Logger.recordOutput("Intake/StopIntake", stopIntake);
    }

    public void toggleCurrentSensing() {
        System.out.println("Current sensing switched to " + (!currentSensing ? "enabled" : "disable"));
        currentSensing = !currentSensing;
    }

    public double getManualMotorPower() {
        return manualPower;
    }

    public double getSensorProximity() {
        return intakeMotor.getOutputCurrent();
    }

    public boolean hasNote() {
        return hasNoteNow;
    }

    public boolean justGotNote() {
        return hasNoteNow && !hadNote;
    }
}
