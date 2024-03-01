package frc.robot.Commands.AutoMove;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.LimelightHelpers;
import frc.robot.Subsystems.Swerve;

public class AutoNoteTrack extends Command{
    
    private Swerve mSwerve;
    
    private PIDController PIDController;

    private double kp;
    private double kd;
    private double ki;

    public AutoNoteTrack(Swerve mSwerve){
        this.mSwerve = mSwerve;

        kp = .05;
        kd = 0;
        ki = 0;

        //addRequirements(mSwerve);
    }

    @Override
    public void initialize() {
        PIDController = new PIDController(kp, ki, kd);
        PIDController.setTolerance(0.5);

        LimelightHelpers.setPipelineIndex("", 0);

    }

    @Override
    public void execute() {
        if(LimelightHelpers.getTV("")){
            double angle = LimelightHelpers.getTX("");
            double speed = PIDController.calculate(angle);

            mSwerve.setDefaultCommand(
                mSwerve.AutoDrive(
                    () -> -Robot.driver.getLeftY(),
                    () -> -Robot.driver.getLeftX(),
                    () -> speed)
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex("", 1);
        mSwerve.setDefaultCommand(
            mSwerve.teleDrive(
            () -> -Robot.driver.getLeftY(), 
            () -> -Robot.driver.getLeftX(), 
            () -> Robot.driver.getRightX()));
    }

}
