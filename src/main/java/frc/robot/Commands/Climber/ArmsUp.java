package frc.robot.Commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Intake;

public class ArmsUp extends Command {

  public Climb Climb;
  public Intake intake;
  private double speed;

  public ArmsUp(Climb mClimb, Intake mIntake, double mspeed) {
    Climb = mClimb;
    speed = mspeed;
    intake = mIntake;
    addRequirements(Climb, intake);
  }

  @Override
  public void initialize() {
    intake.spitPos();
  }

  @Override
  public void execute() {
    Climb.RightUp(speed);
    Climb.LeftUp(speed);
  }

  @Override
  public void end(boolean interrupted) {
    Climb.RightStop();
    Climb.LeftStop();
  }
}
