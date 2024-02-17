package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber.Climber;

public class TeleopIntake extends Command {
    Intake intake;
    private double intakePower = 0;
    public TeleopIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void execute() {
        // if(OI.aButton()){
        //     System.out.println("PRESET");
        //     climber.setElevatorPosition(Constants.Presets.testClimber, 0);
        // }

        intakePower = OI.deadband(OI.getIntakePower(), 0.05);
        if(intakePower != 0 || (intake.getManualMotorPower() != 0 && intakePower == 0)){
            intake.setMotorPower(intakePower);
        }

    }
    
}
