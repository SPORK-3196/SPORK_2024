package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class OI {
    public static ShuffleboardTab X1_Tab = Shuffleboard.getTab("Driver");
    public static ShuffleboardTab X2_Tab = Shuffleboard.getTab("Secondary");
    public static ShuffleboardTab Intake_Tab = Shuffleboard.getTab("Intake");
    public static ShuffleboardTab Shooter_Tab = Shuffleboard.getTab("Shooter");
    
    
    public class kDriver{

        public static boolean a_Button = false;
        public static boolean b_Button = false;
        public static boolean x_Button = false;
        public static boolean y_Button = false;

        public static boolean kRightBumper = false;
        public static boolean kLeftBumper = false;

        public static boolean kStart = false;
        public static boolean kBack = false;

        public static double kRightTrigger = 0;
        public static double kLeftTrigger = 0;

        public static GenericEntry a_Button_Entry = X1_Tab.add("A_Button", false).getEntry();
        public static GenericEntry b_Button_Entry = X1_Tab.add("B_Button", false).getEntry();
        public static GenericEntry x_Button_Entry = X1_Tab.add("X_Button", false).getEntry();
        public static GenericEntry y_Button_Entry = X1_Tab.add("Y_Button", false).getEntry();

        public static GenericEntry kRightBumper_Entry = X1_Tab.add("Right Bumper", false).getEntry();
        public static GenericEntry kLeftBumper_Entry = X1_Tab.add("Left Bumper", false).getEntry();

        public static GenericEntry kBack_Entry = X1_Tab.add("Back", false).getEntry();
        public static GenericEntry kStart_Entry = X1_Tab.add("Start", false).getEntry();

        public static GenericEntry kRightTrigger_Entry = X1_Tab.add("Right trigger", 0.0).getEntry();
        public static GenericEntry kLeftTrigger_Entry = X1_Tab.add("Left trigger", 0.0).getEntry();
    }

    public class kSecondary{

        public static boolean a_Button = false;
        public static boolean b_Button = false;
        public static boolean x_Button = false;
        public static boolean y_Button = false;

        public static boolean kRightBumper = false;
        public static boolean kLeftBumper = false;

        public static boolean kStart = false;
        public static boolean kBack = false;

        public static boolean kRJSD = false;
        public static boolean kLJSD = false;

        public static double kRightTrigger = 0;
        public static double kLeftTrigger = 0;

        public static double kPOV = -1;

        public static GenericEntry a_Button_Entry = X2_Tab.add("A_Button", false).getEntry();
        public static GenericEntry b_Button_Entry = X2_Tab.add("B_Button", false).getEntry();
        public static GenericEntry x_Button_Entry = X2_Tab.add("X_Button", false).getEntry();
        public static GenericEntry y_Button_Entry = X2_Tab.add("Y_Button", false).getEntry();

        public static GenericEntry kRightBumper_Entry = X2_Tab.add("Right Bumper", false).getEntry();
        public static GenericEntry kLeftBumper_Entry = X2_Tab.add("Left Bumper", false).getEntry();

        public static GenericEntry kBack_Entry = X2_Tab.add("Back", false).getEntry();
        public static GenericEntry kStart_Entry = X2_Tab.add("Start", false).getEntry();

        public static GenericEntry kRJSD_Entry = X2_Tab.add("Right joy", false).getEntry();
        public static GenericEntry kLJSD_Entry = X2_Tab.add("Left joy", false).getEntry();

        public static GenericEntry kRightTrigger_Entry = X2_Tab.add("Right trigger", 0.0).getEntry();
        public static GenericEntry kLeftTrigger_Entry = X2_Tab.add("Left trigger", 0.0).getEntry();

        public static GenericEntry kPOV_Entry = X2_Tab.add("D-Pad Value", 0.0).getEntry();
    }

    public static class kShooter{
        public static double ShooterSpeed = 0.0;

        public static GenericEntry kShooterSpeed_Entry = Shooter_Tab.add("Shooter speed", 0.0).getEntry();
    }

    public static class kIntake{
        public static double IntakePos = 0;
        public static double IntakeSpeed = 0;
        public static boolean IntakeRun = false;

        public static GenericEntry kIntakePos_Entry = Intake_Tab.add("Intake Pos", 0.0).getEntry();
        public static GenericEntry kIntakeSpeed_Entry = Intake_Tab.add("Intake Speed", 0.0).getEntry();
        public static GenericEntry kIntakeRun_Entry = Intake_Tab.add("Intake Run", false).getEntry();
    }
}
