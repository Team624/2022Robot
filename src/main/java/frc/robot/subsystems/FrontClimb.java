// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FrontClimb extends SubsystemBase {
  private CANSparkMax frontWinch = new CANSparkMax(Constants.Climb.frontWinchID, MotorType.kBrushless);
  private RelativeEncoder encoder;

  private boolean climbStatus = false;

  private SparkMaxPIDController pidController;

  private double P;
  private double I; 
  private double D; 
  private double Iz; 
  private double FF;
  private double MaxOutput;
  private double MinOutput; 

  private double Pnew;
  private double Inew;
  private double Dnew;
  private double Iznew;
  private double FFnew;

  private ShuffleboardTab tab_frontClimb = Shuffleboard.getTab("Front Climb");

  private NetworkTableEntry Pterm = tab_frontClimb.add("P Term", 0.0).withPosition(1, 0).getEntry();
  private NetworkTableEntry Iterm = tab_frontClimb.add("I Term", 0.0).withPosition(1, 1).getEntry();
  private NetworkTableEntry Dterm = tab_frontClimb.add("D Term", 0.0).withPosition(1, 2).getEntry();
  private NetworkTableEntry IzTerm = tab_frontClimb.add("Iz Term", 0.0).withPosition(1, 3).getEntry();
  private NetworkTableEntry FFterm = tab_frontClimb.add("FF Term", 0.0).withPosition(1, 4).getEntry();

  /** Creates a new frontClimb. */
  public FrontClimb() {
    frontWinch.setIdleMode(IdleMode.kBrake);
    encoder = frontWinch.getEncoder();
    pidController = frontWinch.getPIDController();

    P = Constants.Climb.P;
    I = Constants.Climb.I;
    D = Constants.Climb.D;
    Iz = Constants.Climb.Iz;
    FF = Constants.Climb.FF;
    MaxOutput = Constants.Climb.MaxOutput;
    MinOutput = Constants.Climb.MinOutput;

    pidController.setP(P);
    pidController.setI(I);
    pidController.setD(D);
    pidController.setIZone(Iz);
    pidController.setFF(FF);
    pidController.setOutputRange(MinOutput, MaxOutput);
  }

  @Override
  public void periodic() {
    System.out.println("ENCODER: " + readEncoder());
    checkNT();
    // This method will be called once per scheduler run
  }

  public void checkNT(){
    Pnew = Pterm.getDouble(Constants.Climb.P);
    Inew = Iterm.getDouble(Constants.Climb.I);
    Dnew = Dterm.getDouble(Constants.Climb.D);
    Iznew = IzTerm.getDouble(Constants.Climb.Iz);
    FFnew = FFterm.getDouble(Constants.Climb.FF);

    if((P != Pnew && Pnew != 0.0)) { pidController.setP(Pnew); P = Pnew; }
    if((I != Inew && Inew != 0.0)) { pidController.setI(Inew); I = Inew; }
    if((D != Dnew && Dnew != 0.0)) { pidController.setD(Dnew); D = Dnew; }
    if((Iz != Iznew && Iznew != 0.0)) { pidController.setIZone(Iznew); Iz = Iznew; }
    if((FF != FFnew && FFnew != 0.0)) { pidController.setFF(FFnew); FF = FFnew; }
  }

  public void powerArm(double speed){
    if(climbStatus){
      frontWinch.set(speed);
    }
  }

  public void moveArm(double setPoint){
    if(climbStatus){
      pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
    }
  }

  public void stopMotor(){
    frontWinch.stopMotor();
  }

  public double readEncoder(){
    return encoder.getPosition();
  }

  public void resetClimbStatus(boolean state){
    climbStatus = state;
    encoder.setPosition(0.0);
  }

  public void setClimbStatus(){
    if(climbStatus){
      climbStatus = false;
    }else{
      climbStatus = true;
    }
  }
  
  public void resetEncoder(){
    encoder.setPosition(0.0);
  }

  public boolean getStatus(){
    return climbStatus;
  }

}
