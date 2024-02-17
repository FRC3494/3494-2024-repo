package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Elevator.Elevator;

public class TeleopElevator extends Command {
    Elevator elevator;
    private double elevatorPower = 0;
    public TeleopElevator(Elevator elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void execute() {
        // if(OI.aButton()){
        //     System.out.println("PRESET");
        //     elevator.setElevatorPosition(Constants.Presets.testElevator, 0);
        // }

        elevatorPower = OI.deadband(OI.getElevatorPower(), 0.05);
        if(elevatorPower != 0 || (elevator.getManualMotorPower() != 0 && elevatorPower == 0)){
            elevator.setElevatorPower(elevatorPower);
        }

    }
    
}
