package frc.robot.Commands.Rollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Roller;

public class RollersRun extends Command {

  private Roller mRoller;

  public RollersRun(Roller mRoller) {
    this.mRoller = mRoller;

    addRequirements(mRoller);
  }

  @Override
  public void initialize() {
    mRoller.RunRoller();
  }

  @Override
  public void end(boolean interrupted) {
    mRoller.StopRoller();
  }
}
