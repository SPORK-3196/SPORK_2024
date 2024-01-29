// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Climber.ArmsDown;
import frc.robot.Commands.Climber.ArmsUp;
import frc.robot.Commands.Intake.RunIntake;
import frc.robot.Constants.kClimber;
import frc.robot.OI.kDriver;
import frc.robot.OI.kSecondary;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Roller;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // Subsystem Initalising 
  public Swerve mSwerve = new Swerve();
  public Climb mClimb = new Climb();
  public Intake mIntake = new Intake();
  public Roller mRoller = new Roller();
  public Shooter mShooter = new Shooter();
  
  public static AHRS gyro = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kMXP);
  
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

  @Override
  public void robotInit() {
    mSwerve.setDefaultCommand(
      mSwerve.teleDrive(
      () -> driver.getLeftY(), 
      () -> driver.getLeftX(), 
      () -> driver.getRightX()));  
    configureBindings();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Driver SmartDashboard output
    if(driver.isConnected()){
      kDriver.a_Button = driver.getAButton();
      kDriver.b_Button = driver.getBButton();
      kDriver.x_Button = driver.getXButton();
      kDriver.y_Button = driver.getYButton();

      kDriver.kBack = driver.getBackButton();
      kDriver.kStart = driver.getStartButton();

      kDriver.kRightBumper = driver.getRightBumper();
      kDriver.kLeftBumper = driver.getLeftBumper();

      kDriver.kRightTrigger = driver.getRightTriggerAxis();
      kDriver.kLeftTrigger = driver.getLeftTriggerAxis();
    }

    // Secondary SmartDashboard output
    if(secondary.isConnected()){
      kSecondary.a_Button = secondary.getAButton();
      kSecondary.b_Button = secondary.getBButton();
      kSecondary.x_Button = secondary.getXButton();
      kSecondary.y_Button = secondary.getYButton();

      kSecondary.kBack = secondary.getBackButton();
      kSecondary.kStart = secondary.getStartButton();

      kSecondary.kRJSD = secondary.getRightStickButton();
      kSecondary.kLJSD = secondary.getLeftStickButton();

      kSecondary.kRightBumper = secondary.getRightBumper();
      kSecondary.kLeftBumper = secondary.getLeftBumper();

      kSecondary.kRightTrigger = secondary.getRightTriggerAxis();
      kSecondary.kLeftTrigger = secondary.getLeftTriggerAxis();

      kSecondary.kPOV = secondary.getPOV();
    }

    // stream to SmartDashboard if DriverStation is not in game
    if (!DriverStation.isFMSAttached()) {
      kDriver.a_Button_Entry.setBoolean(kDriver.a_Button);
      kDriver.b_Button_Entry.setBoolean(kDriver.b_Button);
      kDriver.x_Button_Entry.setBoolean(kDriver.x_Button);
      kDriver.y_Button_Entry.setBoolean(kDriver.y_Button);

      kDriver.kBack_Entry.setBoolean(kDriver.kBack);
      kDriver.kStart_Entry.setBoolean(kDriver.kStart);

      kDriver.kLeftBumper_Entry.setBoolean(kDriver.kLeftBumper);
      kDriver.kRightBumper_Entry.setBoolean(kDriver.kRightBumper);

      kDriver.kLeftTrigger_Entry.setDouble(kDriver.kLeftTrigger);
      kDriver.kRightTrigger_Entry.setDouble(kDriver.kRightTrigger);


      kSecondary.a_Button_Entry.setBoolean(kSecondary.a_Button);
      kSecondary.b_Button_Entry.setBoolean(kSecondary.b_Button);
      kSecondary.x_Button_Entry.setBoolean(kSecondary.x_Button);
      kSecondary.y_Button_Entry.setBoolean(kSecondary.y_Button);

      kSecondary.kBack_Entry.setBoolean(kSecondary.kBack);
      kSecondary.kStart_Entry.setBoolean(kSecondary.kStart);

      kSecondary.kRJSD_Entry.setBoolean(kSecondary.kRJSD);
      kSecondary.kLJSD_Entry.setBoolean(kSecondary.kLJSD);

      kSecondary.kLeftBumper_Entry.setBoolean(kSecondary.kLeftBumper);
      kSecondary.kRightBumper_Entry.setBoolean(kSecondary.kRightBumper);

      kSecondary.kLeftTrigger_Entry.setDouble(kSecondary.kLeftTrigger);
      kSecondary.kRightTrigger_Entry.setDouble(kSecondary.kRightTrigger);

      kSecondary.kPOV_Entry.setDouble(kSecondary.kPOV);
    }


    // shows both outside and durring a game 


    SmartDashboard.putNumber("gyro angle", gyro.getYaw());
    SmartDashboard.putNumber("FR angle",Swerve.FR.getCANforshuffle().getRotations());
    SmartDashboard.putNumber("BR angle",Swerve.BR.getCANforshuffle().getRotations());
    SmartDashboard.putNumber("FL angle",Swerve.FL.getCANforshuffle().getRotations());
    SmartDashboard.putNumber("BL angle",Swerve.BL.getCANforshuffle().getRotations());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

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

    

    // Secondary Button Bindings
    secondary_Left_Trigger.whileTrue(new ArmsDown(mClimb, kClimber.ClimbSpeed));
    secondary_Right_Trigger.whileTrue(new ArmsUp(mClimb, kClimber.ClimbSpeed));

    secondary_a_Button.onTrue(new RunIntake(mIntake, mShooter));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }


}
