// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private ShuffleboardTab tab = Shuffleboard.getTab("Climb");
  private NetworkTableEntry centerSpeed = tab.add("Center Speed", Constants.Climb.centerWinchSpeed).withPosition(0, 0).getEntry();
  private NetworkTableEntry armSpeed = tab.add("Arm Speed", Constants.Climb.armWinchSpeed).withPosition(0, 1).getEntry();
  private NetworkTableEntry upDelay = tab.add("Up Delay", Constants.Climb.upperLowerdelay).withPosition(0, 2).getEntry();

  private CANSparkMax centerWinchSpark = new CANSparkMax(Constants.Climb.centerWinchMotorID, MotorType.kBrushless);
  private CANSparkMax armWinchSpark = new CANSparkMax(Constants.Climb.armWinchMotorID, MotorType.kBrushless);
  private PneumaticHub hub;
  private Solenoid bottomSolenoid;
  private Solenoid upperSolenoid;

  private Timer timer;

  private boolean climbStatus = false;
  private boolean timerStatus = false;
  private boolean isTimerRunning = false;

  /** Creates a new Climb. */
  public Climb(PneumaticHub hub) {
    timer = new Timer();
    centerWinchSpark.setIdleMode(IdleMode.kBrake);
    armWinchSpark.setIdleMode(IdleMode.kBrake);
    this.hub = hub;
    bottomSolenoid = this.hub.makeSolenoid(Constants.Climb.bottomLeftPistonID);
    upperSolenoid = this.hub.makeSolenoid(Constants.Climb.topLeftPistonID);
  }

  @Override
  public void periodic() {
    setTimer();
    checkTimer();
  }

  public void setMode(){
    if(climbStatus){
      climbStatus = false;
    }else{
      climbStatus = true;
    }
  }

  public void actuateSet(){
    if(climbStatus){
      timerStatus = true;
      upperSolenoid.set(true);
    }
  }

  private void setTimer(){
    if(timerStatus){
      if(!isTimerRunning){
        timer.reset();
        timer.start();
        isTimerRunning = true;
      }
    }else{
      isTimerRunning = false;
    }
  }

  private void checkTimer(){
    if(timer.get() > upDelay.getDouble(Constants.Climb.upperLowerdelay) && !bottomSolenoid.get()){
      bottomSolenoid.set(true);
      timer.stop();
      timer.reset();
      timerStatus = false;
    }
  }

  public void actuateLowerPistons(){
    if(climbStatus){
      bottomSolenoid.set(true);
    }
  }

  public void retractLowerPistons(){
    if(climbStatus){
      bottomSolenoid.set(false);
    }
  }

  public void actuateUpperPistons(){
    if(climbStatus){
      upperSolenoid.set(true);
    }
  }

  public void retractUpperPistons(){
    if(climbStatus){
      upperSolenoid.set(false);
    }
  }

  public void extendCenterWinch(){
    if(climbStatus){
      centerWinchSpark.set(centerSpeed.getDouble(0.0));
    }
  }



  public void retractCenterWinch(){
    if(climbStatus){
      centerWinchSpark.set(-centerSpeed.getDouble(0.0));
    }
  }

  public void stopCenterWinch(){
    if(climbStatus){
      centerWinchSpark.stopMotor(); 
    }
  }

  public void extendArmWinch(){
    if(climbStatus){
      armWinchSpark.set(armSpeed.getDouble(0.0)); 
    }
  }

  public void retractArmWinch(){
    if(climbStatus){
      armWinchSpark.set(-armSpeed.getDouble(0.0));
    }
  }

  public void stopArmWinch(){
    if(climbStatus){
      armWinchSpark.stopMotor();
    }
  }
}
