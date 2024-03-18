package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kShooter;
import frc.robot.Subsystems.Shooter;

public class RunShooter extends Command {

  public Shooter shooter;

  public RunShooter(Shooter shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setShooterSpeed(kShooter.ShooterSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.ShooterIdle();
  }
}
