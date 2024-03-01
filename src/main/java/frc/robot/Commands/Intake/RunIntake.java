package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kIntake;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class RunIntake extends Command{
    
    private Intake mIntake;

    public RunIntake(Intake intake){
        mIntake = intake;

        addRequirements(mIntake);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        // TODO 10 is a stand in number must change
        if(mIntake.IntakeEncoder.getPosition() > kIntake.FloorPickup - 5 ){
            mIntake.grab();
        }else if(mIntake.IntakeEncoder.getPosition() < kIntake.ShooterPos + 6){
            mIntake.feed();
        }else{
            mIntake.Keep();
        }
    }

    @Override
    public void end(boolean interrupted) {
        mIntake.Keep();
    }

}
