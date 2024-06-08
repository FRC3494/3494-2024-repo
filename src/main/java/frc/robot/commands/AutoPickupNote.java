package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class AutoPickupNote extends Command {
    private Timer timer;
    Drivetrain drivetrain;
    Intake intake;
    private double time;

    public AutoPickupNote(Drivetrain drivetrain, Intake intake, double time) {
        this.time = time;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.timer = new Timer();
        this.timer.start();
        addRequirements(drivetrain);
        addRequirements(intake);
    }

    // DOCUMENT SPEED: work slow was: -0.5, and motor torque was 0.3
    @Override
    public void execute() {
        double driveSpeed = -2.0;
        if (drivetrain.seesNote() == false) {
            driveSpeed = 0;
        }
        System.out.println(time + "|" + timer.hasElapsed(time));
        drivetrain.drive(0.0, driveSpeed,
                -drivetrain.getNoteRotationPower(), false);
    }

    @Override
    public void end(boolean interupted) {
        System.out.println("ENDING Note COMMAND");
        drivetrain.drive(0.0, 0.2,
                0.0, false);
    }

    @Override
    public boolean isFinished() {
        // if (timer.hasElapsed(time)) {
        // return true;
        // }
        if (intake.hasNote()) {
            return true;
        }
        return false;

    }
}
