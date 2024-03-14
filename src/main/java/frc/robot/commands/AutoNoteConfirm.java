package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class AutoNoteConfirm extends Command {
    Intake intake;


    public AutoNoteConfirm(Intake intake) {
        this.intake = intake;        
    }

    @Override
    public boolean isFinished(){
        if(intake.hasNote()) return true;
        return false;
        
    } 
}
