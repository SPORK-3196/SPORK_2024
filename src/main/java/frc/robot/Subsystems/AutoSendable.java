package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kAuto;

public class AutoSendable {
    private final Swerve swerve;

    public AutoBuilder builder;

    public SendableChooser<Command> Auto = AutoBuilder.buildAutoChooser();

    public AutoSendable(Swerve swerve,AutoBuilder builder){
        this.swerve = swerve;
        this.builder = builder;

        
        AutoBuilder.buildAuto("simple Forward Turn");
        
        AutoBuilder.buildAutoChooser();
    }

    public Command getChosenCommand(){
        return Auto.getSelected();
    }
}
