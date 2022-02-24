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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private ShuffleboardTab tab = Shuffleboard.getTab("Climb");
  private NetworkTableEntry centerSpeed = tab.add("Center Speed", 0.0).withPosition(0, 0).getEntry();
  private NetworkTableEntry armSpeed = tab.add("Arm Speed", 0.0).withPosition(0, 1).getEntry();

  private CANSparkMax centerWinchSpark = new CANSparkMax(Constants.Climb.centerWinchMotorID, MotorType.kBrushless);
  private CANSparkMax armWinchSpark = new CANSparkMax(Constants.Climb.armWinchMotorID, MotorType.kBrushless);
  private PneumaticHub hub;
  private Solenoid bottomSolenoid;
  private Solenoid upperSolenoid;


  private boolean climbStatus = false;

  /** Creates a new Climb. */
  public Climb(PneumaticHub hub) {
    centerWinchSpark.setIdleMode(IdleMode.kBrake);
    armWinchSpark.setIdleMode(IdleMode.kBrake);
    this.hub = hub;
    bottomSolenoid = this.hub.makeSolenoid(10);
    upperSolenoid = this.hub.makeSolenoid(11);
  }

  public void activateClimb(){
    climbStatus = true;
  }

  public void deactiveClimb(){
    climbStatus = false;
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
      centerWinchSpark.set(-centerSpeed.getDouble(0.0));
    }
  }

  public void retractCenterWinch(){
    if(climbStatus){
      centerWinchSpark.set(centerSpeed.getDouble(0.0));
    }
  }

  public void stopCenterWinch(){
    if(climbStatus){
      centerWinchSpark.stopMotor(); 
    }
  }

  public void extendArmWinch(){
    if(climbStatus){
      armWinchSpark.set(-armSpeed.getDouble(0.0)); 
    }
  }

  public void retractArmWinch(){
    if(climbStatus){
      armWinchSpark.set(armSpeed.getDouble(0.0));
    }
  }

  public void stopArmWinch(){
    if(climbStatus){
      armWinchSpark.stopMotor();
    }
  }
}
