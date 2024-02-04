package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PathFollowingController;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kSwerve;

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
        () -> {RunPath(PathPlannerPath.fromPathFile("Path"));}, swerve));
    }

    public Command getpath(){
        return this.PathChooser.getSelected();
    }

    public Command RunPath(PathPlannerPath Path){
        return this.runOnce(() -> 
        new FollowPathHolonomic(
        Path,
        swerve::getPose,
        swerve::getChassisSpeedsRR,
        (speeds) -> swerve.DriveRR(speeds),
        new HolonomicPathFollowerConfig(
            new PIDConstants(2,0,0.1),
            new PIDConstants(2, 0, 0.1),
            kSwerve.MaxSpeed,
            Units.inchesToMeters(12),
            new ReplanningConfig(false, false)),
        () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
            return false;
        },
        swerve));
    }
}
