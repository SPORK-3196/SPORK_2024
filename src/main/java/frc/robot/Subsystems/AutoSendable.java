package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kAuto;

public class AutoSendable extends SubsystemBase {
    private final Swerve swerve;

    public SendableChooser<Command> PathChooser = new SendableChooser<>();
    public SendableChooser<Command> Auto = new SendableChooser<>();


    public AutoSendable(Swerve swerve){
        this.swerve = swerve;

        AutoBuilder.configureHolonomic(null, null, null, null, null, null, swerve);
        SmartDashboard.putData("Auto Chooser", Auto);
        SmartDashboard.putData("path Chooser", PathChooser);
        SmartDashboard.putBoolean("Auto Builder Config", AutoBuilder.isConfigured());
        SmartDashboard.putBoolean("is pathfinding config", AutoBuilder.isPathfindingConfigured());
        SetUpPaths();
    }

    public void SetUpPaths(){
        PathChooser.setDefaultOption("Nothing", null);
        PathChooser.addOption("Simple Forward Turn", Commands.runOnce(() -> {RunPath(PathPlannerPath.fromPathFile("path"), false);}));
    }

    public Command getpath(){
        return PathChooser.getSelected();
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

    public Command getChosenCommand(){
        return Auto.getSelected();
    }
}
