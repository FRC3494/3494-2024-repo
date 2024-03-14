package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class AutoPickupNote extends Command {
    private XboxController rumbler;
    private Timer timer;
    Drivetrain drivetrain;
    Intake intake;
    private double time;


    public AutoPickupNote(Drivetrain drivetrain, Intake intake, double time) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.timer =  new Timer();
        this.timer.start();
        addRequirements(drivetrain);
        addRequirements(intake);
        
    }
    @Override
    public void execute(){
        drivetrain.drive( 0.0,0.2,
        -drivetrain.getNoteRotationPower(), false);
    }
    @Override
    public boolean isFinished(){
        if(timer.hasElapsed(time)){return true;}
        // else if(intake.JustgotNote()){return true;}
        return false;
        
    } 
}
