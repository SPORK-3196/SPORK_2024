package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kRollerBars;
import frc.robot.Constants.kShooter;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Roller;
import frc.robot.Subsystems.Shooter;

public class RunAmp extends Command {

  private Shooter mShooter;
  private Intake mIntake;
  private Roller mRoller;

  public RunAmp(Shooter mShooter, Intake mIntake, Roller mRoller) {
    this.mShooter = mShooter;
    this.mIntake = mIntake;
    this.mRoller = mRoller;

    addRequirements(mShooter, mIntake, mRoller);
  }

  @Override
  public void initialize() {
    mRoller.RollerDown();
    mRoller.RunRoller();
  }

  @Override
  public void execute() {
    if (mRoller.getRollerPos() >= kRollerBars.RollerRefDown - 2) {
      mShooter.setShooterSpeed(kShooter.ampSpeed);
      mIntake.feed();
    }
  }

  @Override
  public void end(boolean interrupted) {
    mRoller.RollerUp();
    mRoller.StopRoller();
    mShooter.ShooterIdle();
    mIntake.Keep();
  }
}
