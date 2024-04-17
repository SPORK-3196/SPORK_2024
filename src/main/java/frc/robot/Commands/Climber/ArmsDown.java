package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Intake;

public class ArmsDown extends Command {

  public Climb Climb;
  public Intake intake;
  private double speed;

  public ArmsDown(Climb Climb,Intake mIntake, double speed) {
    this.Climb = Climb;
    this.speed = speed;
    intake = mIntake;

    addRequirements(Climb, intake);
  }

  @Override
  public void initialize() {
    intake.ClimbPos();
  }
  
  @Override
  public void execute() {
    Climb.LeftDown(speed);
    Climb.RightDown(speed);
  }

  @Override
  public void end(boolean interrupted) {
    Climb.LeftStop();
    Climb.RightStop();
  }
}
