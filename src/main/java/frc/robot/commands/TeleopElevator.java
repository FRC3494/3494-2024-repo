package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Elevator.Elevator;

public class TeleopElevator extends Command {
    private Elevator elevator;
    private double elevatorPower = 0;

    public TeleopElevator(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevatorPower = OI.deadband(OI.getElevatorPower(), 0.05);

        if (elevatorPower != 0 || (elevator.getManualMotorPower() != 0 && elevatorPower == 0)) {
            elevator.setElevatorPower(elevatorPower);
        }
    }

}
