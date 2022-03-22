// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private ShuffleboardTab tab = Shuffleboard.getTab("Climb");
  private NetworkTableEntry armSpeed = tab.add("Arm Speed", Constants.Climb.armWinchSpeed).withPosition(0, 0).getEntry();
  private NetworkTableEntry holdSpeed = tab.add("Hold Speed", Constants.Climb.armWinchSpeed).withPosition(0, 0).getEntry();

  private CANSparkMax frontWinch = new CANSparkMax(Constants.Climb.frontWinchID, MotorType.kBrushless);
  private CANSparkMax backWinch = new CANSparkMax(Constants.Climb.backWinchID, MotorType.kBrushless);

  private boolean climbStatus = false;
  public boolean frontHoldStatus = false;
  public boolean backHoldStatus = false;

  /** Creates a new Climb. */
  public Climb() {
    frontWinch.setIdleMode(IdleMode.kBrake);
    backWinch.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {

  }

  public void setMode(){
    if(climbStatus){
      climbStatus = false;
    }else{
      climbStatus = true;
    }
  }

  public void setFrontHold(){
    if(frontHoldStatus){
      frontHoldStatus = false;
    }else{
      frontHoldStatus = true;
    }
  }

  public void setBackHold(){
    if(backHoldStatus){
      backHoldStatus = false;
    }else{
      backHoldStatus = true;
    }
  }

  public void resetMode(){
    climbStatus = false;
  }

  public void powerFrontArm(double speed){
    if(frontHoldStatus){
      speed = getHoldSpeed();
    }else{
      frontHoldStatus = false;
    }
    frontWinch.set(speed);
  }

  public void powerBackArm(double speed){
    if(backHoldStatus){
      speed = getHoldSpeed();
    }else{
      backHoldStatus = false;
    }
    backWinch.set(-speed);
  }

  public void stopFrontArm(){
    frontWinch.stopMotor();
  }

  public void stopBackArm(){
    backWinch.stopMotor();
  }

  public double getHoldSpeed(){

      return Constants.Climb.holdSpeed;
  }

}