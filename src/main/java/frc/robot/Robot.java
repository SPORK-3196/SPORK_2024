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
  public XboxController driver = new XboxController(0);
  public XboxController secondary = new XboxController(1);

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



}