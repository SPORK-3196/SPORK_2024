package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb;

public class ArmsDown extends Command {

  public Climb Climb;
  private double speed;

  public ArmsDown(Climb Climb, double speed) {
    this.Climb = Climb;
    this.speed = speed;

    addRequirements(Climb);
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
