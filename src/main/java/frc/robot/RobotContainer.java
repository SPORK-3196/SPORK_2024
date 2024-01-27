// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.kShooter;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Roller;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;

public class RobotContainer {

  // Subsystem Initalising 
  public Swerve mSwerve = new Swerve();
  public Intake mIntake = new Intake();
  public Roller mRoller = new Roller();
  public Shooter mShooter = new Shooter();

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

  public RobotContainer() {

    mSwerve.setDefaultCommand(
    mSwerve.teleDrive(
    () -> driver.getLeftY(), 
    () -> driver.getLeftX(), 
    () -> driver.getRightX()));

    configureBindings();
  }

  private void configureBindings() {    
    driver_a_Button.toggleOnTrue(new InstantCommand(() -> mSwerve.ZeroGyro(), mSwerve));
    driver_b_Button.toggleOnTrue(new InstantCommand(() -> mShooter.RunShooter(0.5)));  // replace with constant val for testing 

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
