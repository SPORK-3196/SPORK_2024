package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;

public class AutoSendable extends SubsystemBase{
    public Swerve swerve;

    public SendableChooser<PathPlannerPath> PathChooser = new SendableChooser<>();
    public SendableChooser<PathPlannerPath> AutoChooser = new SendableChooser<>();

    public PathPlannerPath path;


    public AutoSendable(Swerve swerve){
        this.swerve = swerve;
        SetUpPaths();

        SmartDashboard.putData("Auto Chooser", AutoChooser);
        SmartDashboard.putData("path Chooser", PathChooser);
        SmartDashboard.putBoolean("Auto Builder Config", AutoBuilder.isConfigured());
        SmartDashboard.putBoolean("is pathfinding config", AutoBuilder.isPathfindingConfigured());
    }

    public void SetUpAutos(){
        // AutoChooser.setDefaultOption("Nothing", null);
    }

    public void SetUpPaths(){
        PathChooser.addOption("Forward", PathPlannerPath.fromPathFile("Forward"));
        PathChooser.addOption("2_note", PathPlannerPath.fromPathFile("2_note"));
        // PathChooser.addOption("Simple arc", PathPlannerPath.fromPathFile("simple arc"));
    }

    public Command getpath(){

        return AutoBuilder.followPath(PathChooser.getSelected());
    }
}
