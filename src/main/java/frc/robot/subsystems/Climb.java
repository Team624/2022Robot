// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private PWMSparkMax winchSpark = new PWMSparkMax(Constants.Climb.winchMotorID);
  private Solenoid bottomLeft = new Solenoid(PneumaticsModuleType.REVPH, Constants.Climb.bottomLeftPistonID);
  private Solenoid bottomRight = new Solenoid(PneumaticsModuleType.REVPH, Constants.Climb.bottomRightPistonID);
  private Solenoid topLeft = new Solenoid(PneumaticsModuleType.REVPH, Constants.Climb.topLeftPistonID);
  private Solenoid topRight = new Solenoid(PneumaticsModuleType.REVPH, Constants.Climb.topRightPistonID);

  private boolean climbStatus = false;

  /** Creates a new Climb. */
  public Climb() {}

  @Override
  public void periodic() {
    //System.out.println(climbStatus);
    // This method will be called once per scheduler run
  }

  public void activateClimb(){
    climbStatus = true;
  }

  public void deactiveClimb(){
    climbStatus = false;
  }

  public void actuateBottomPistons(){}

  public void actuateUpperPistons(){}

  public void extendCenterWinch(){}

  public void retractCenterWinch(){}

  public void extendArmWinch(){
    System.out.println("Extending Arm Winch");
  }

  public void retractArmWinch(){
    System.out.println("Retracting Arm Winch");
  }
}
