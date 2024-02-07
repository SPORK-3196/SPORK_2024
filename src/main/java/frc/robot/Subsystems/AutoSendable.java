package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSendable extends SubsystemBase{
    public Swerve swerve;

    public SendableChooser<Command> AutoChooser = AutoBuilder.buildAutoChooser();


    public AutoSendable(Swerve swerve){
        this.swerve = swerve;
        SetUpNamedCommands();
        SetUpPaths();

        SmartDashboard.putData("path Chooser", AutoChooser);
        SmartDashboard.putBoolean("Auto Builder Config", AutoBuilder.isConfigured());
        SmartDashboard.putBoolean("is pathfinding config", AutoBuilder.isPathfindingConfigured());
    }

    public void SetUpNamedCommands(){
        //NamedCommands.registerCommand("Zero gyro", Commands.runOnce(() -> swerve.ZeroGyro()));
    }

    public void SetUpPaths(){
        AutoChooser.addOption("Forward", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Forward")));
        AutoChooser.addOption("2_note", AutoBuilder.followPath(PathPlannerPath.fromPathFile("2_note")));
        AutoChooser.addOption("Square", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Square")));
        // AutoChooser.addOption("Foretold", PathPlannerPath.fromPathFile("ForeTold"));
    }

    public Command getpath(){
        return AutoChooser.getSelected();
    }
}
