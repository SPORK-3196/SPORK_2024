// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lighting{
  public Spark Lighting = new Spark(9);

// Using REV Blinkin (Such a fun name)

  public Lighting(){
    Lighting.set(0.87);
  }

  public void setRed(){
    Lighting.set(0.61);
  }

  public void setBlue(){
    Lighting.set(0.87);
  } 
  
  public void setRedRunUP(){
    Lighting.set(0.87);
  }

  public void setBlueRunUp(){
    Lighting.set(0.87);
  }

  public void setRedRunDown(){
    Lighting.set(0.87);
  }

  public void setBlueRunDown(){
    Lighting.set(0.87);
  }

}