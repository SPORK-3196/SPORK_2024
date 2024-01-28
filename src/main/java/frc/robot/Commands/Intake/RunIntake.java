package frc.robot.Commands.Intake;

import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Subsystems.Intake;

public class RunIntake extends Command{
    
    public Intake mIntake;


    public RunIntake(Intake intake){
        mIntake = intake;
    
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        // TODO 10 is a stand in number must change
        if(!mIntake.NoteIn.isPressed() && mIntake.IntakeEncoder.getPosition() < 10 ){
            mIntake.grab();
        }else if(mIntake.NoteIn.isPressed() && mIntake.IntakeEncoder.getPosition() > 10){
            mIntake.feed();
        }else{
            mIntake.Keep();
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
