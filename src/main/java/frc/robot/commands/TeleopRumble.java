package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopRumble extends Command {
    private XboxController rumbler;
    private Timer timer;
    public TeleopRumble(XboxController rumbler){
        this.rumbler = rumbler;
        this.timer =  new Timer();
        this.timer.start();
    }
    @Override
    public void execute(){
        rumbler.setRumble(RumbleType.kBothRumble, 1.0);
    }
    @Override
    public boolean isFinished(){
        if(timer.hasElapsed(0.1)){return true;}
        return false;
    } 
}
