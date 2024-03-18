package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class IntakeFeed extends Command {

  private Intake intake;

  public IntakeFeed(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.feed();
  }

  @Override
  public void end(boolean interrupted) {
    intake.Keep();
  }
}
