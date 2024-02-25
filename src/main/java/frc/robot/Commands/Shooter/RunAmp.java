package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kShooter;
import frc.robot.Subsystems.Shooter;

public class RunAmp extends Command {

private Shooter mShooter;

    public RunAmp(Shooter mShooter){
        this.mShooter = mShooter;

        addRequirements(mShooter);
    }

    @Override
    public void initialize() {
        mShooter.setShooterSpeed(kShooter.ampSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.ShooterIdle();
    }
}
