package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class TeleopDriveAutomated extends CommandBase {
    private Timer timer;
    Drivetrain drivetrain;
    Intake intake;
    private double time;
    private double xPow;
    private double yPow;


    public TeleopDriveAutomated(Drivetrain drivetrain, double xPow, double yPow, double time) {
        this.drivetrain = drivetrain;
        
        this.timer =  new Timer();
        this.timer.start();
        addRequirements(drivetrain);
        this.xPow = xPow;
        this.yPow = yPow;
        
    }
    @Override
    public void execute(){
        drivetrain.drive(xPow,yPow,
        0.0, false);
    }
    @Override
    public boolean isFinished(){
        return true;
        
    } 
}
