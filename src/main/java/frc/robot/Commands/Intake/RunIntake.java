package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class RunIntake extends Command{
    
    public Intake mIntake;
    public Shooter mShooter;

    public RunIntake(Intake intake, Shooter shooter){
        mIntake = intake;
        mShooter = shooter;

        addRequirements(mIntake, mShooter);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        // TODO only feed when the shooter is up to speed - could be driver controlled?

        // TODO 10 is a stand in number must change
        if(!mIntake.NoteIn.isPressed() && mIntake.IntakeEncoder.getPosition() < 10 ){
            mIntake.grab();
        }else if(mIntake.NoteIn.isPressed() && mIntake.IntakeEncoder.getPosition() > 10 && mShooter.isShooterToSpeed()){
            mIntake.feed();
        }else{
            mIntake.Keep();
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
