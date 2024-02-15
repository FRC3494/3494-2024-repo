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
        climberPower = OI.deadband(OI.getClimberPower(), 0.05);
        climber.setElevatorPower(climberPower);

    }
    
}
