package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Climber.Climber;

public class TeleopClimber extends Command {
    Climber climber;
    private double climberPower = 0;
    public TeleopClimber(Climber climber){
        this.climber = climber;
        addRequirements(climber);
    }
    @Override
    public void execute() {
        // if(OI.aButton()){
        //     System.out.println("PRESET");
        //     climber.setElevatorPosition(Constants.Presets.testClimber, 0);
        // }
        
        climberPower = OI.deadband(OI.getClimberPower(), 0.05);
        if(climberPower <= 0){
            climberPower *= 0.8;
        }
        if(climberPower != 0 || (climber.getManualMotorPower() != 0 && climberPower == 0)){
            climber.setElevatorPower(climberPower);
        }

    }
    
}
