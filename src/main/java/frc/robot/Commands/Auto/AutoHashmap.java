package frc.robot.Commands.Auto;

import java.util.LinkedHashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Swerve;

public class AutoHashmap {
    @SuppressWarnings("unused")
    private final Swerve swerve;

    private final LinkedHashMap<String, PathPlannerPath> path = 
        new LinkedHashMap<String, PathPlannerPath>();
    private final LinkedHashMap<String, Command> Autos = 
        new LinkedHashMap<String, Command>();
    private final SendableChooser<Command> sendableChooser = 
        new SendableChooser<Command>();

    public AutoHashmap(Swerve swerve){
        this.swerve = swerve;

        LoadPaths();
        loadActions();
        LoadAutos();
        populateSendable();
    }

    
    private void LoadPaths(){
        path.put("Path", PathPlannerPath.fromPathFile("Path"));
    }

    private void loadActions(){

    }

    private void LoadAutos(){
        Autos.put("No Auto", Commands.waitSeconds(0));
    }

    private void populateSendable(){
        sendableChooser.setDefaultOption("No Auto", Autos.get("No Auto"));
            for(Map.Entry<String, Command> entry : Autos.entrySet()){
                sendableChooser.addOption(entry.getKey(), entry.getValue());
            }
    }

    public SendableChooser<Command> getSendable(){
        return sendableChooser;
    }







}
