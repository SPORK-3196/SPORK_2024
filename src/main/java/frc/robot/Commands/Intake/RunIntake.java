package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kIntake;
import frc.robot.Subsystems.Intake;

// old intake command from UNC Pembroke

public class RunIntake extends Command {

  private Intake mIntake;

  public RunIntake(Intake intake) {
    mIntake = intake;

    addRequirements(mIntake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (mIntake.IntakeEncoder.getPosition() > kIntake.FloorPickup - 5) {
      mIntake.grab();
    } else if (mIntake.IntakeEncoder.getPosition() < kIntake.ShooterPos + 6) {
      mIntake.feed();
    } else {
      mIntake.Keep();
    }
  }

  @Override
  public void end(boolean interrupted) {
    mIntake.Keep();
  }
}
