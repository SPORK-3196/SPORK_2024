package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kAuto;

public class AutoSendable extends SubsystemBase {
    private final Swerve swerve;

    public SendableChooser<Command> PathChooser = new SendableChooser<>();
    public SendableChooser<Command> Auto = AutoBuilder.buildAutoChooser("No Auto");


    public AutoSendable(Swerve swerve){
        this.swerve = swerve;

        SmartDashboard.putData("Auto Chooser", Auto);
        SmartDashboard.putData("path Chooser", PathChooser);
        SmartDashboard.putBoolean("Auto Builder Config", AutoBuilder.isConfigured());
        SmartDashboard.putBoolean("is pathfinding config", AutoBuilder.isPathfindingConfigured());

        AutoBuilder.buildAuto("simple Forward Turn");
    }

    public Command RunPath(PathPlannerPath Path, boolean Alliance){
        return this.runOnce(() -> 
        new FollowPathHolonomic(
        Path,
        swerve::getPose,
        swerve::getChassisSpeeds,
        (speeds) -> swerve.Drive(speeds),
        kAuto.AutoConfig,
        () -> Alliance,
        swerve));
    }

    public Command getChosenCommand(){
        return Auto.getSelected();
    }
}
