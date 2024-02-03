package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kAuto;

public class AutoSendable extends SubsystemBase{
    public Swerve swerve;

    public SendableChooser<Command> PathChooser = new SendableChooser<Command>();
    public SendableChooser<Command> AutoChooser = new SendableChooser<Command>();


    public AutoSendable(Swerve swerve){
        this.swerve = swerve;
        SetUpPaths();

        SmartDashboard.putData("Auto Chooser", AutoChooser);
        SmartDashboard.putData("path Chooser", PathChooser);
        SmartDashboard.putBoolean("Auto Builder Config", AutoBuilder.isConfigured());
        SmartDashboard.putBoolean("is pathfinding config", AutoBuilder.isPathfindingConfigured());
    }

    public void SetUpAutos(){
        AutoChooser.setDefaultOption("Nothing", Commands.waitSeconds(0));
    }

    public void SetUpPaths(){
        PathChooser.setDefaultOption("Nothing", Commands.waitSeconds(0));
        
        PathChooser.addOption("Simple Forward", Commands.runOnce(
        () -> {RunPath(PathPlannerPath.fromPathFile("Path"), false);}, swerve));
    }

    public Command getpath(){
        return this.PathChooser.getSelected();
    }

    public Command RunPath(PathPlannerPath Path, boolean Allience){
        return this.runOnce(() -> 
        new FollowPathHolonomic(
        Path,
        swerve::getPose,
        swerve::getChassisSpeedsRR,
        (speeds) -> swerve.DriveRR(speeds),
        kAuto.AutoConfig,
        () -> Allience,
        swerve));
    }
}
