// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.LEDcolors;

public class Lighting extends SubsystemBase{
  public double mColor;
  public Spark Lighting = new Spark(1);

// Using REV Blinkin (Such a fun name)

  public Lighting(){
    mColor = 0;
    Lighting = new Spark(9);
  }

  @Override
  public void periodic() {
    Lighting.set(mColor);
  }

  public void ChangeColor(LEDcolors color){ 
    switch (color) {
      case kNoteIn:
        mColor = 0;
        break;
      case kIntakeRunning:
        mColor = 0;
        break;
      case kNoAllience:
        mColor = 0;
        break;
      case kBlueAllience:
        mColor = 0;
        break;
      case kRedAllience:
        mColor = 0;
        break;
    }
  }
}