package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.LEDcolors;
import frc.robot.Robot;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Lighting;

public class AutoIntake extends Command {

  private Intake mIntake;
  private Lighting mLighting;

  public AutoIntake(Intake mIntake, Lighting mLighting) {
    this.mIntake = mIntake;
    this.mLighting = mLighting;

    addRequirements(mIntake, mLighting);
  }

  @Override
  public void initialize() {
    mIntake.FloorPos();
    mIntake.grab();
  }

  @Override
  public void execute() {
    // dont plz
    if(Robot.NoteIN) {
    
    }else{
      Robot.secondary.setRumble(RumbleType.kBothRumble, 1);
      Robot.driver.setRumble(RumbleType.kBothRumble, 0.5);
      mIntake.ShooterPos();
    }
  }

  @Override
  public void end(boolean interrupted) {
    mIntake.ShooterPos();
    mIntake.Keep();
    var Alliance = DriverStation.getAlliance();
    if (Alliance.isPresent()) {
      if (Alliance.get() == DriverStation.Alliance.Red) {
        mLighting.ChangeColor(LEDcolors.kRedAllience);
      } else {
        mLighting.ChangeColor(LEDcolors.kBlueAllience);
      }
    }
    Robot.secondary.setRumble(RumbleType.kBothRumble, 0);
    Robot.driver.setRumble(RumbleType.kBothRumble, 0);
  }
}
