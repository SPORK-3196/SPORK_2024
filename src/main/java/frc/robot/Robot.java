// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Climber.ArmsDown;
import frc.robot.Commands.Climber.ArmsUp;
import frc.robot.Commands.Intake.RunIntake;
import frc.robot.Commands.Intake.Vomit;
import frc.robot.Commands.Shooter.RunAmp;
import frc.robot.Commands.Shooter.RunShooter;
import frc.robot.Constants.kClimber;
import frc.robot.OI.oDriver;
import frc.robot.OI.oIntake;
import frc.robot.OI.oSecondary;
import frc.robot.OI.oShooter;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Lighting;
import frc.robot.Subsystems.Roller;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static AHRS gyro = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kMXP);


  // Subsystem Initalising 
  public SendableChooser<Command> autoChooser;
  public static Swerve mSwerve = new Swerve();
  public static Roller mRoller = new Roller();
  public static Climb mClimb = new Climb();
  public static Intake mIntake = new Intake();
  public static Shooter mShooter = new Shooter();
  public static Lighting mLighting = new Lighting();
  
  // Controllers  
  public static XboxController driver = new XboxController(0);
  public static XboxController secondary = new XboxController(1);

  // Driver buttons
  public JoystickButton driver_a_Button = new JoystickButton(driver, XboxController.Button.kA.value);
  public JoystickButton driver_b_Button = new JoystickButton(driver, XboxController.Button.kB.value);
  public JoystickButton driver_x_Button = new JoystickButton(driver, XboxController.Button.kX.value);
  public JoystickButton driver_y_Button = new JoystickButton(driver, XboxController.Button.kY.value);

  // Driver bumpers and triggers
  public JoystickButton driver_left_Bumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  public JoystickButton driver_Right_Bumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  public JoystickButton driver_Left_Trigger = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
  public JoystickButton driver_Right_Trigger = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);

  // Driver misc buttons
  public JoystickButton driver_Start = new JoystickButton(driver, XboxController.Button.kStart.value);
  public JoystickButton driver_Back = new JoystickButton(driver, XboxController.Button.kBack.value); 

  // Secondary buttons
  public JoystickButton secondary_a_Button = new JoystickButton(secondary, XboxController.Button.kA.value);
  public JoystickButton secondary_b_Button = new JoystickButton(secondary, XboxController.Button.kB.value);
  public JoystickButton secondary_x_Button = new JoystickButton(secondary, XboxController.Button.kX.value);
  public JoystickButton secondary_y_Button = new JoystickButton(secondary, XboxController.Button.kY.value);

  // Secondary bumpers and triggers
  public JoystickButton secondary_left_Bumper = new JoystickButton(secondary, XboxController.Button.kLeftBumper.value);
  public JoystickButton secondary_Right_Bumper = new JoystickButton(secondary, XboxController.Button.kRightBumper.value);
  public JoystickButton secondary_Left_Trigger = new JoystickButton(secondary, XboxController.Axis.kLeftTrigger.value);
  public JoystickButton secondary_Right_Trigger = new JoystickButton(secondary, XboxController.Axis.kRightTrigger.value);
  
  // Secondary misc buttons
  public JoystickButton secondary_Start = new JoystickButton(secondary, XboxController.Button.kStart.value);
  public JoystickButton secondary_Back = new JoystickButton(secondary, XboxController.Button.kBack.value);

  public JoystickButton secondary_RJSD = new JoystickButton(secondary, XboxController.Button.kRightStick.value);
  public JoystickButton secondary_LJSD = new JoystickButton(secondary, XboxController.Button.kLeftStick.value);

  // Camera
  UsbCamera Cam = CameraServer.startAutomaticCapture(0);

  @Override
  public void robotInit() {
    mSwerve.setDefaultCommand(
      mSwerve.teleDrive(
      () -> -driver.getLeftY(), 
      () -> -driver.getLeftX(), 
      () -> driver.getRightX()));

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser("simple Forward Turn");
    SmartDashboard.putData("Auto", autoChooser);

    Cam.setFPS(28);
    Cam.setResolution(144, 144);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Driver SmartDashboard output
    if(driver.isConnected()){
      oDriver.a_Button = driver.getAButton();
      oDriver.b_Button = driver.getBButton();
      oDriver.x_Button = driver.getXButton();
      oDriver.y_Button = driver.getYButton();

      oDriver.kBack = driver.getBackButton();
      oDriver.kStart = driver.getStartButton();

      oDriver.kRightBumper = driver.getRightBumper();
      oDriver.kLeftBumper = driver.getLeftBumper();

      oDriver.kRightTrigger = driver.getRightTriggerAxis();
      oDriver.kLeftTrigger = driver.getLeftTriggerAxis();
    }

    // Secondary SmartDashboard output
    if(secondary.isConnected()){
      oSecondary.a_Button = secondary.getAButton();
      oSecondary.b_Button = secondary.getBButton();
      oSecondary.x_Button = secondary.getXButton();
      oSecondary.y_Button = secondary.getYButton();

      oSecondary.kBack = secondary.getBackButton();
      oSecondary.kStart = secondary.getStartButton();

      oSecondary.kRJSD = secondary.getRightStickButton();
      oSecondary.kLJSD = secondary.getLeftStickButton();

      oSecondary.kRightBumper = secondary.getRightBumper();
      oSecondary.kLeftBumper = secondary.getLeftBumper();

      oSecondary.kRightTrigger = secondary.getRightTriggerAxis();
      oSecondary.kLeftTrigger = secondary.getLeftTriggerAxis();

      oSecondary.kPOV = secondary.getPOV();
    }

    // stream to SmartDashboard if DriverStation is not in game
    if (!DriverStation.isFMSAttached()) {
      oDriver.a_Button_Entry.setBoolean(oDriver.a_Button);
      oDriver.b_Button_Entry.setBoolean(oDriver.b_Button);
      oDriver.x_Button_Entry.setBoolean(oDriver.x_Button);
      oDriver.y_Button_Entry.setBoolean(oDriver.y_Button);

      oDriver.kBack_Entry.setBoolean(oDriver.kBack);
      oDriver.kStart_Entry.setBoolean(oDriver.kStart);

      oDriver.kLeftBumper_Entry.setBoolean(oDriver.kLeftBumper);
      oDriver.kRightBumper_Entry.setBoolean(oDriver.kRightBumper);

      oDriver.kLeftTrigger_Entry.setDouble(oDriver.kLeftTrigger);
      oDriver.kRightTrigger_Entry.setDouble(oDriver.kRightTrigger);


      oSecondary.a_Button_Entry.setBoolean(oSecondary.a_Button);
      oSecondary.b_Button_Entry.setBoolean(oSecondary.b_Button);
      oSecondary.x_Button_Entry.setBoolean(oSecondary.x_Button);
      oSecondary.y_Button_Entry.setBoolean(oSecondary.y_Button);

      oSecondary.kBack_Entry.setBoolean(oSecondary.kBack);
      oSecondary.kStart_Entry.setBoolean(oSecondary.kStart);

      oSecondary.kRJSD_Entry.setBoolean(oSecondary.kRJSD);
      oSecondary.kLJSD_Entry.setBoolean(oSecondary.kLJSD);

      oSecondary.kLeftBumper_Entry.setBoolean(oSecondary.kLeftBumper);
      oSecondary.kRightBumper_Entry.setBoolean(oSecondary.kRightBumper);

      oSecondary.kLeftTrigger_Entry.setDouble(oSecondary.kLeftTrigger);
      oSecondary.kRightTrigger_Entry.setDouble(oSecondary.kRightTrigger);

      oSecondary.kPOV_Entry.setDouble(oSecondary.kPOV);
    }

    if (!DriverStation.isFMSAttached()) {
      oIntake.IntakePos = mIntake.getPos();
      oIntake.IntakeRun = !mIntake.isRunning();
      oIntake.IntakeSpeed = mIntake.getSpeed();
      oIntake.LimitDown = mIntake.FloorStop.isPressed();
      oIntake.LimitUp = mIntake.SpeakerLimit.isPressed();
      oIntake.NoteIn = mIntake.NoteIn.get();

      oShooter.ShooterSpeed = mShooter.getShooterSpeed();

      oIntake.kIntakePos_Entry.setDouble(oIntake.IntakePos);
      oIntake.kIntakeRun_Entry.setBoolean(oIntake.IntakeRun);
      oIntake.kIntakeSpeed_Entry.setDouble(oIntake.IntakeSpeed);
      oIntake.kLimitDown.setBoolean(oIntake.LimitDown);
      oIntake.kLimitUp.setBoolean(oIntake.LimitUp);
      oIntake.kNoteIn.setBoolean(oIntake.NoteIn);

      oShooter.kShooterSpeed_Entry.setDouble(oShooter.ShooterSpeed);
    }

    SmartDashboard.putNumber("gyro angle", gyro.getYaw());


    // if (mIntake.SpeakerLimit.isPressed()) {
    //   mIntake.IntakeEncoder.setPosition(0);
    // }

    // if (mIntake.FloorStop.isPressed()) {
    //   mIntake.Stop();
    // }
  }

  @Override
  public void disabledInit() {
    // ranbow run in disabled
  }

  @Override
  public void disabledPeriodic() {
    // turn to alliance color if connected 

    
    // if (isRed()) {
    //   mLighting.setRed();
    // }else{
    //   mLighting.setBlue();
    // }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();

    if(!(m_autonomousCommand == null)){
    m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if(!(m_autonomousCommand == null)){
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

      // Intake position
    if (secondary.getPOV() == 180) {
      mIntake.FloorPos();
    }

    if(secondary.getPOV() == 0){
      mIntake.ShooterPos();
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void configureBindings() {    

    // Driver Button Bindings
    driver_a_Button.toggleOnTrue(new InstantCommand(() -> mSwerve.ZeroGyro(), mSwerve));
    //driver_x_Button.whileTrue(new InstantCommand(() -> mSwerve.Xconfig(), mSwerve));

    // Secondary Button Bindings

      // Scoring
    secondary_b_Button.whileTrue(new RunShooter(mShooter));
    secondary_RJSD.whileTrue(new RunAmp(mShooter));
    secondary_x_Button.whileTrue(new Vomit(mIntake));
    secondary_a_Button.whileTrue(new RunIntake(mIntake, mShooter));

      // Climber
    secondary_left_Bumper.whileTrue(new ArmsDown(mClimb, kClimber.ClimbSpeed));
    secondary_Right_Bumper.whileTrue(new ArmsUp(mClimb, kClimber.ClimbSpeed));


  }

  public boolean isRed(){
    var Alliance = DriverStation.getAlliance();
    if(Alliance.isPresent()){
        return Alliance.get() == DriverStation.Alliance.Red;
    }
        return false;
   }

public class LimelightHelpers {

    public static class LimelightTarget_Retro {

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private  double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace()
        {
            return toPose3D(cameraPose_TargetSpace);
        }
        public Pose3d getRobotPose_FieldSpace()
        {
            return toPose3D(robotPose_FieldSpace);
        }
        public Pose3d getRobotPose_TargetSpace()
        {
            return toPose3D(robotPose_TargetSpace);
        }
        public Pose3d getTargetPose_CameraSpace()
        {
            return toPose3D(targetPose_CameraSpace);
        }
        public Pose3d getTargetPose_RobotSpace()
        {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D()
        {
            return toPose2D(cameraPose_TargetSpace);
        }
        public Pose2d getRobotPose_FieldSpace2D()
        {
            return toPose2D(robotPose_FieldSpace);
        }
        public Pose2d getRobotPose_TargetSpace2D()
        {
            return toPose2D(robotPose_TargetSpace);
        }
        public Pose2d getTargetPose_CameraSpace2D()
        {
            return toPose2D(targetPose_CameraSpace);
        }
        public Pose2d getTargetPose_RobotSpace2D()
        {
            return toPose2D(targetPose_RobotSpace);
        }

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("ts")
        public double ts;

        public LimelightTarget_Retro() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }

    }

    public static class LimelightTarget_Fiducial {

        @JsonProperty("fID")
        public double fiducialID;

        @JsonProperty("fam")
        public String fiducialFamily;

        @JsonProperty("t6c_ts")
        private double[] cameraPose_TargetSpace;

        @JsonProperty("t6r_fs")
        private double[] robotPose_FieldSpace;

        @JsonProperty("t6r_ts")
        private double[] robotPose_TargetSpace;

        @JsonProperty("t6t_cs")
        private double[] targetPose_CameraSpace;

        @JsonProperty("t6t_rs")
        private double[] targetPose_RobotSpace;

        public Pose3d getCameraPose_TargetSpace()
        {
            return toPose3D(cameraPose_TargetSpace);
        }
        public Pose3d getRobotPose_FieldSpace()
        {
            return toPose3D(robotPose_FieldSpace);
        }
        public Pose3d getRobotPose_TargetSpace()
        {
            return toPose3D(robotPose_TargetSpace);
        }
        public Pose3d getTargetPose_CameraSpace()
        {
            return toPose3D(targetPose_CameraSpace);
        }
        public Pose3d getTargetPose_RobotSpace()
        {
            return toPose3D(targetPose_RobotSpace);
        }

        public Pose2d getCameraPose_TargetSpace2D()
        {
            return toPose2D(cameraPose_TargetSpace);
        }
        public Pose2d getRobotPose_FieldSpace2D()
        {
            return toPose2D(robotPose_FieldSpace);
        }
        public Pose2d getRobotPose_TargetSpace2D()
        {
            return toPose2D(robotPose_TargetSpace);
        }
        public Pose2d getTargetPose_CameraSpace2D()
        {
            return toPose2D(targetPose_CameraSpace);
        }
        public Pose2d getTargetPose_RobotSpace2D()
        {
            return toPose2D(targetPose_RobotSpace);
        }
        
        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        @JsonProperty("ts")
        public double ts;
        
        public LimelightTarget_Fiducial() {
            cameraPose_TargetSpace = new double[6];
            robotPose_FieldSpace = new double[6];
            robotPose_TargetSpace = new double[6];
            targetPose_CameraSpace = new double[6];
            targetPose_RobotSpace = new double[6];
        }
    }

    public static class LimelightTarget_Barcode {

    }

    public static class LimelightTarget_Classifier {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("zone")
        public double zone;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        public  LimelightTarget_Classifier() {
        }
    }

    public static class LimelightTarget_Detector {

        @JsonProperty("class")
        public String className;

        @JsonProperty("classID")
        public double classID;

        @JsonProperty("conf")
        public double confidence;

        @JsonProperty("ta")
        public double ta;

        @JsonProperty("tx")
        public double tx;

        @JsonProperty("txp")
        public double tx_pixels;

        @JsonProperty("ty")
        public double ty;

        @JsonProperty("typ")
        public double ty_pixels;

        public LimelightTarget_Detector() {
        }
    }

    public static class Results {

        @JsonProperty("pID")
        public double pipelineID;

        @JsonProperty("tl")
        public double latency_pipeline;

        @JsonProperty("cl")
        public double latency_capture;

        public double latency_jsonParse;

        @JsonProperty("ts")
        public double timestamp_LIMELIGHT_publish;

        @JsonProperty("ts_rio")
        public double timestamp_RIOFPGA_capture;

        @JsonProperty("v")
        @JsonFormat(shape = Shape.NUMBER)
        public boolean valid;

        @JsonProperty("botpose")
        public double[] botpose;

        @JsonProperty("botpose_wpired")
        public double[] botpose_wpired;

        @JsonProperty("botpose_wpiblue")
        public double[] botpose_wpiblue;

        @JsonProperty("botpose_tagcount")
        public double botpose_tagcount;
       
        @JsonProperty("botpose_span")
        public double botpose_span;
       
        @JsonProperty("botpose_avgdist")
        public double botpose_avgdist;
       
        @JsonProperty("botpose_avgarea")
        public double botpose_avgarea;

        @JsonProperty("t6c_rs")
        public double[] camerapose_robotspace;

        public Pose3d getBotPose3d() {
            return toPose3D(botpose);
        }
    
        public Pose3d getBotPose3d_wpiRed() {
            return toPose3D(botpose_wpired);
        }
    
        public Pose3d getBotPose3d_wpiBlue() {
            return toPose3D(botpose_wpiblue);
        }

        public Pose2d getBotPose2d() {
            return toPose2D(botpose);
        }
    
        public Pose2d getBotPose2d_wpiRed() {
            return toPose2D(botpose_wpired);
        }
    
        public Pose2d getBotPose2d_wpiBlue() {
            return toPose2D(botpose_wpiblue);
        }

        @JsonProperty("Retro")
        public LimelightTarget_Retro[] targets_Retro;

        @JsonProperty("Fiducial")
        public LimelightTarget_Fiducial[] targets_Fiducials;

        @JsonProperty("Classifier")
        public LimelightTarget_Classifier[] targets_Classifier;

        @JsonProperty("Detector")
        public LimelightTarget_Detector[] targets_Detector;

        @JsonProperty("Barcode")
        public LimelightTarget_Barcode[] targets_Barcode;

        public Results() {
            botpose = new double[6];
            botpose_wpired = new double[6];
            botpose_wpiblue = new double[6];
            camerapose_robotspace = new double[6];
            targets_Retro = new LimelightTarget_Retro[0];
            targets_Fiducials = new LimelightTarget_Fiducial[0];
            targets_Classifier = new LimelightTarget_Classifier[0];
            targets_Detector = new LimelightTarget_Detector[0];
            targets_Barcode = new LimelightTarget_Barcode[0];

        }
    }

    public static class LimelightResults {
        @JsonProperty("Results")
        public Results targetingResults;
        
        public String error;

        public LimelightResults() {
            targetingResults = new Results();
            error = "";
        }


    }

    public static class PoseEstimate {
        public Pose2d pose;
        public double timestampSeconds;
        public double latency;
        public int tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;


        public PoseEstimate(Pose2d pose, double timestampSeconds, double latency, int tagCount, double tagSpan, double avgTagDist, double avgTagArea) {
            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.latency = latency;
            this.tagCount = tagCount;
            this.tagSpan = tagSpan;
            this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea;
        }
    }

    private static ObjectMapper mapper;

    /**
     * Print JSON Parse time to the console in milliseconds
     */
    static boolean profileJSON = false;

    static final String sanitizeName(String name) {
        if (name == "" || name == null) {
            return "limelight";
        }
        return name;
    }

    private static Pose3d toPose3D(double[] inData){
        if(inData.length < 6)
        {
            //System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                    Units.degreesToRadians(inData[5])));
    }

    private static Pose2d toPose2D(double[] inData){
        if(inData.length < 6)
        {
            //System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }
        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }

    private static double extractBotPoseEntry(double[] inData, int position){
        if(inData.length < position+1)
        {
            return 0;
        }
        return inData[position];
    }

    private static PoseEstimate getBotPoseEstimate(String limelightName, String entryName) {
        var poseEntry = LimelightHelpers.getLimelightNTTableEntry(limelightName, entryName);
        var poseArray = poseEntry.getDoubleArray(new double[0]);
        var pose = toPose2D(poseArray);
        double latency = extractBotPoseEntry(poseArray,6);
        int tagCount = (int)extractBotPoseEntry(poseArray,7);
        double tagSpan = extractBotPoseEntry(poseArray,8);
        double tagDist = extractBotPoseEntry(poseArray,9);
        double tagArea = extractBotPoseEntry(poseArray,10);
        //getlastchange() in microseconds, ll latency in milliseconds
        var timestamp = (poseEntry.getLastChange() / 1000000.0) - (latency/1000.0);
        return new PoseEstimate(pose, timestamp,latency,tagCount,tagSpan,tagDist,tagArea);
    }

    public static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
    }

    public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    public static double getLimelightNTDouble(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    }

    public static void setLimelightNTDouble(String tableName, String entryName, double val) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(val);
    }

    public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
        getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
    }

    public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
    }

    public static String getLimelightNTString(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getString("");
    }

    public static URL getLimelightURLString(String tableName, String request) {
        String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
        URL url;
        try {
            url = new URL(urlString);
            return url;
        } catch (MalformedURLException e) {
            System.err.println("bad LL URL");
        }
        return null;
    }
    /////
    /////

    public static double getTX(String limelightName) {
        return getLimelightNTDouble(limelightName, "tx");
    }

    public static double getTY(String limelightName) {
        return getLimelightNTDouble(limelightName, "ty");
    }

    public static double getTA(String limelightName) {
        return getLimelightNTDouble(limelightName, "ta");
    }

    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightNTDouble(limelightName, "tl");
    }

    public static double getLatency_Capture(String limelightName) {
        return getLimelightNTDouble(limelightName, "cl");
    }

    public static double getCurrentPipelineIndex(String limelightName) {
        return getLimelightNTDouble(limelightName, "getpipe");
    }

    public static String getJSONDump(String limelightName) {
        return getLimelightNTString(limelightName, "json");
    }

    /**
     * Switch to getBotPose
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    /**
     * Switch to getBotPose_wpiRed
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    /**
     * Switch to getBotPose_wpiBlue
     * 
     * @param limelightName
     * @return
     */
    @Deprecated
    public static double[] getBotpose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    public static double[] getBotPose(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    public static double[] getBotPose_wpiRed(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    public static double[] getBotPose_wpiBlue(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    public static double[] getBotPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
    }

    public static double[] getCameraPose_TargetSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
    }

    public static double[] getTargetPose_CameraSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
    }

    public static double[] getTargetPose_RobotSpace(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    }

    public static double[] getTargetColor(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "tc");
    }

    public static double getFiducialID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tid");
    }

    public static double getNeuralClassID(String limelightName) {
        return getLimelightNTDouble(limelightName, "tclass");
    }

    /////
    /////

    public static Pose3d getBotPose3d(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose");
        return toPose3D(poseArray);
    }

    public static Pose3d getBotPose3d_wpiRed(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpired");
        return toPose3D(poseArray);
    }

    public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
        return toPose3D(poseArray);
    }

    public static Pose3d getBotPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_targetspace");
        return toPose3D(poseArray);
    }

    public static Pose3d getCameraPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
        return toPose3D(poseArray);
    }

    public static Pose3d getTargetPose3d_CameraSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
        return toPose3D(poseArray);
    }

    public static Pose3d getTargetPose3d_RobotSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
        return toPose3D(poseArray);
    }

    public static Pose3d getCameraPose3d_RobotSpace(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_robotspace");
        return toPose3D(poseArray);
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param limelightName
     * @return
     */
    public static Pose2d getBotPose2d_wpiBlue(String limelightName) {

        double[] result = getBotPose_wpiBlue(limelightName);
        return toPose2D(result);
    }

    /**
     * Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when you are on the BLUE
     * alliance
     * 
     * @param limelightName
     * @return
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpiblue");
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param limelightName
     * @return
     */
    public static Pose2d getBotPose2d_wpiRed(String limelightName) {

        double[] result = getBotPose_wpiRed(limelightName);
        return toPose2D(result);

    }

    /**
     * Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when you are on the RED
     * alliance
     * @param limelightName
     * @return
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpired");
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     * 
     * @param limelightName
     * @return
     */
    public static Pose2d getBotPose2d(String limelightName) {

        double[] result = getBotPose(limelightName);
        return toPose2D(result);

    }

    public static boolean getTV(String limelightName) {
        return 1.0 == getLimelightNTDouble(limelightName, "tv");
    }

    /////
    /////

    public static void setPipelineIndex(String limelightName, int pipelineIndex) {
        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
    }

    
    public static void setPriorityTagID(String limelightName, int ID) {
        setLimelightNTDouble(limelightName, "priorityid", ID);
    }

    /**
     * The LEDs will be controlled by Limelight pipeline settings, and not by robot
     * code.
     */
    public static void setLEDMode_PipelineControl(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 0);
    }

    public static void setLEDMode_ForceOff(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    public static void setLEDMode_ForceBlink(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 2);
    }

    public static void setLEDMode_ForceOn(String limelightName) {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }

    public static void setStreamMode_Standard(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 0);
    }

    public static void setStreamMode_PiPMain(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 1);
    }

    public static void setStreamMode_PiPSecondary(String limelightName) {
        setLimelightNTDouble(limelightName, "stream", 2);
    }

    public static void setCameraMode_Processor(String limelightName) {
        setLimelightNTDouble(limelightName, "camMode", 0);
    }
    public static void setCameraMode_Driver(String limelightName) {
        setLimelightNTDouble(limelightName, "camMode", 1);
    }


    /**
     * Sets the crop window. The crop window in the UI must be completely open for
     * dynamic cropping to work.
     */
    public static void setCropWindow(String limelightName, double cropXMin, double cropXMax, double cropYMin, double cropYMax) {
        double[] entries = new double[4];
        entries[0] = cropXMin;
        entries[1] = cropXMax;
        entries[2] = cropYMin;
        entries[3] = cropYMax;
        setLimelightNTDoubleArray(limelightName, "crop", entries);
    }

    public static void setCameraPose_RobotSpace(String limelightName, double forward, double side, double up, double roll, double pitch, double yaw) {
        double[] entries = new double[6];
        entries[0] = forward;
        entries[1] = side;
        entries[2] = up;
        entries[3] = roll;
        entries[4] = pitch;
        entries[5] = yaw;
        setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries);
    }

    /////
    /////

    public static void setPythonScriptData(String limelightName, double[] outgoingPythonData) {
        setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData);
    }

    public static double[] getPythonScriptData(String limelightName) {
        return getLimelightNTDoubleArray(limelightName, "llpython");
    }

    /////
    /////

    /**
     * Asynchronously take snapshot.
     */
    public static CompletableFuture<Boolean> takeSnapshot(String tableName, String snapshotName) {
        return CompletableFuture.supplyAsync(() -> {
            return SYNCH_TAKESNAPSHOT(tableName, snapshotName);
        });
    }

    private static boolean SYNCH_TAKESNAPSHOT(String tableName, String snapshotName) {
        URL url = getLimelightURLString(tableName, "capturesnapshot");
        try {
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            if (snapshotName != null && snapshotName != "") {
                connection.setRequestProperty("snapname", snapshotName);
            }

            int responseCode = connection.getResponseCode();
            if (responseCode == 200) {
                return true;
            } else {
                System.err.println("Bad LL Request");
            }
        } catch (IOException e) {
            System.err.println(e.getMessage());
        }
        return false;
    }

    /**
     * Parses Limelight's JSON results dump into a LimelightResults Object
     */
    public static LimelightResults getLatestResults(String limelightName) {

        long start = System.nanoTime();
        LimelightHelpers.LimelightResults results = new LimelightHelpers.LimelightResults();
        if (mapper == null) {
            mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
        }

        try {
            results = mapper.readValue(getJSONDump(limelightName), LimelightResults.class);
        } catch (JsonProcessingException e) {
            results.error = "lljson error: " + e.getMessage();
        }

        long end = System.nanoTime();
        double millis = (end - start) * .000001;
        results.targetingResults.latency_jsonParse = millis;
        if (profileJSON) {
            System.out.printf("lljson: %.2f\r\n", millis);
        }

        return results;
    }
}

}