package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class OI {

  public static ShuffleboardTab X1_Tab = Shuffleboard.getTab("Driver");
  public static ShuffleboardTab X2_Tab = Shuffleboard.getTab("Secondary");
  public static ShuffleboardTab Intake_Tab = Shuffleboard.getTab("Intake");
  public static ShuffleboardTab Shooter_Tab = Shuffleboard.getTab("Shooter");
  public static ShuffleboardTab PDH_Tab = Shuffleboard.getTab("PDH");

  public class oDriver {

    public static boolean a_Button = false;
    public static boolean b_Button = false;
    public static boolean x_Button = false;
    public static boolean y_Button = false;

    public static boolean kRightBumper = false;
    public static boolean kLeftBumper = false;

    public static boolean kStart = false;
    public static boolean kBack = false;

    public static double kRightTrigger = 0.0;
    public static double kLeftTrigger = 0.0;

    public static GenericEntry a_Button_Entry = X1_Tab
      .add("A_Button", false)
      .getEntry();
    public static GenericEntry b_Button_Entry = X1_Tab
      .add("B_Button", false)
      .getEntry();
    public static GenericEntry x_Button_Entry = X1_Tab
      .add("X_Button", false)
      .getEntry();
    public static GenericEntry y_Button_Entry = X1_Tab
      .add("Y_Button", false)
      .getEntry();

    public static GenericEntry kRightBumper_Entry = X1_Tab
      .add("Right Bumper", false)
      .getEntry();
    public static GenericEntry kLeftBumper_Entry = X1_Tab
      .add("Left Bumper", false)
      .getEntry();

    public static GenericEntry kBack_Entry = X1_Tab
      .add("Back", false)
      .getEntry();
    public static GenericEntry kStart_Entry = X1_Tab
      .add("Start", false)
      .getEntry();

    public static GenericEntry kRightTrigger_Entry = X1_Tab
      .add("Right trigger", 0.0)
      .getEntry();
    public static GenericEntry kLeftTrigger_Entry = X1_Tab
      .add("Left trigger", 0.0)
      .getEntry();
  }

  public class oSecondary {

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

    public static double kRightTrigger = 0.0;
    public static double kLeftTrigger = 0.0;

    public static double kPOV = -1;

    public static GenericEntry a_Button_Entry = X2_Tab
      .add("A_Button", false)
      .getEntry();
    public static GenericEntry b_Button_Entry = X2_Tab
      .add("B_Button", false)
      .getEntry();
    public static GenericEntry x_Button_Entry = X2_Tab
      .add("X_Button", false)
      .getEntry();
    public static GenericEntry y_Button_Entry = X2_Tab
      .add("Y_Button", false)
      .getEntry();

    public static GenericEntry kRightBumper_Entry = X2_Tab
      .add("Right Bumper", false)
      .getEntry();
    public static GenericEntry kLeftBumper_Entry = X2_Tab
      .add("Left Bumper", false)
      .getEntry();

    public static GenericEntry kBack_Entry = X2_Tab
      .add("Back", false)
      .getEntry();
    public static GenericEntry kStart_Entry = X2_Tab
      .add("Start", false)
      .getEntry();

    public static GenericEntry kRJSD_Entry = X2_Tab
      .add("Right joy", false)
      .getEntry();
    public static GenericEntry kLJSD_Entry = X2_Tab
      .add("Left joy", false)
      .getEntry();

    public static GenericEntry kRightTrigger_Entry = X2_Tab
      .add("Right trigger", 0.0)
      .getEntry();
    public static GenericEntry kLeftTrigger_Entry = X2_Tab
      .add("Left trigger", 0.0)
      .getEntry();

    public static GenericEntry kPOV_Entry = X2_Tab
      .add("D-Pad Value", 0.0)
      .getEntry();
  }

  public static class oShooter {

    public static double ShooterSpeed = 0.0;

    public static GenericEntry kShooterSpeed_Entry = Shooter_Tab
      .add("Shooter speed", 0.0)
      .getEntry();
  }

  public static class oIntake {

    public static double IntakePos = 0;
    public static double IntakeSpeed = 0;
    public static boolean IntakeRun = false;
    public static boolean NoteIn = false;
    public static boolean LimitDown = false;
    public static boolean LimitUp = false;

    public static GenericEntry kNoteInEntry = Intake_Tab
      .add("Note In", false)
      .getEntry();
    public static GenericEntry kIntakePos_Entry = Intake_Tab
      .add("Intake Pos", 0.0)
      .getEntry();
    public static GenericEntry kIntakeSpeed_Entry = Intake_Tab
      .add("Intake Speed", 0.0)
      .getEntry();
    public static GenericEntry kIntakeRun_Entry = Intake_Tab
      .add("Intake Run", false)
      .getEntry();
    public static GenericEntry kLimitDown = Intake_Tab
      .add("Intake Down", false)
      .getEntry();
    public static GenericEntry kLimitUp = Intake_Tab
      .add("Intake Up", false)
      .getEntry();
  }

  public static class oPDH{
    public static double Channel_0 = 0.0;
    public static double Channel_1 = 0.0;
    public static double Channel_2 = 0.0;
    public static double Channel_3 = 0.0;
    public static double Channel_4 = 0.0;
    public static double Channel_5 = 0.0;
    public static double Channel_6 = 0.0;
    public static double Channel_7 = 0.0;
    public static double Channel_8 = 0.0;
    public static double Channel_9 = 0.0;
    public static double Channel_10 = 0.0;
    public static double Channel_11 = 0.0;
    public static double Channel_12 = 0.0;
    public static double Channel_13 = 0.0;
    public static double Channel_14 = 0.0;
    public static double Channel_15 = 0.0;
    public static double Channel_16 = 0.0;
    public static double Channel_17 = 0.0;
    public static double Channel_18 = 0.0;
    public static double Channel_19 = 0.0;
    public static double Channel_20 = 0.0;
    public static double Channel_21 = 0.0;
    public static double Channel_22 = 0.0;
    public static double Channel_23 = 0.0;
    

    public static GenericEntry kChannel_0 = PDH_Tab.add("Channel_0", 0.0).getEntry();
    public static GenericEntry kChannel_1 = PDH_Tab.add("Channel_1", 0.0).getEntry();
    public static GenericEntry kChannel_2 = PDH_Tab.add("Channel_2", 0.0).getEntry();
    public static GenericEntry kChannel_3 = PDH_Tab.add("Channel_3", 0.0).getEntry();
    public static GenericEntry kChannel_4 = PDH_Tab.add("Channel_4", 0.0).getEntry();
    public static GenericEntry kChannel_5 = PDH_Tab.add("Channel_5", 0.0).getEntry();
    public static GenericEntry kChannel_6 = PDH_Tab.add("Channel_6",0.0).getEntry();
    public static GenericEntry kChannel_7 = PDH_Tab.add("Channel_7",0.0).getEntry();
    public static GenericEntry kChannel_8 = PDH_Tab.add("Channel_8", 0.0).getEntry();
    public static GenericEntry kChannel_9 = PDH_Tab.add("Channel_9", 0.0).getEntry();
    public static GenericEntry kChannel_10 = PDH_Tab.add("Channel_10", 0.0).getEntry();
    public static GenericEntry kChannel_11 = PDH_Tab.add("Channel_11", 0.0).getEntry();
    public static GenericEntry kChannel_12 = PDH_Tab.add("Channel_12", 0.0).getEntry();
    public static GenericEntry kChannel_13 = PDH_Tab.add("Channel_13", 0.0).getEntry();
    public static GenericEntry kChannel_14 = PDH_Tab.add("Channel_14", 0.0).getEntry();
    public static GenericEntry kChannel_15 = PDH_Tab.add("Channel_15", 0.0).getEntry();
    public static GenericEntry kChannel_16 = PDH_Tab.add("Channel_16", 0.0).getEntry();
    public static GenericEntry kChannel_17 = PDH_Tab.add("Channel_17", 0.0).getEntry();
    public static GenericEntry kChannel_18 = PDH_Tab.add("Channel_18", 0.0).getEntry();
    public static GenericEntry kChannel_19 = PDH_Tab.add("Channel_19", 0.0).getEntry();
    public static GenericEntry kChannel_20 = PDH_Tab.add("Channel_20", 0.0).getEntry();
    public static GenericEntry kChannel_21 = PDH_Tab.add("Channel_21", 0.0).getEntry();
    public static GenericEntry kChannel_22 = PDH_Tab.add("Channel_22", 0.0).getEntry();
    public static GenericEntry kChannel_23 = PDH_Tab.add("Channel_23", 0.0).getEntry();
  }
}