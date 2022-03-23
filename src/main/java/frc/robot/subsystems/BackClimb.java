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

public class BackClimb extends SubsystemBase {
  private CANSparkMax backWinch = new CANSparkMax(Constants.Climb.backWinchID, MotorType.kBrushless);
  private RelativeEncoder encoder;
  private SparkMaxPIDController pidController;

  private boolean climbStatus = false;

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

  private ShuffleboardTab tab_backClimb = Shuffleboard.getTab("Back Climb");

  private NetworkTableEntry Pterm = tab_backClimb.add("P Term", 0.0).withPosition(1, 0).getEntry();
  private NetworkTableEntry Iterm = tab_backClimb.add("I Term", 0.0).withPosition(1, 1).getEntry();
  private NetworkTableEntry Dterm = tab_backClimb.add("D Term", 0.0).withPosition(1, 2).getEntry();
  private NetworkTableEntry IzTerm = tab_backClimb.add("Iz Term", 0.0).withPosition(1, 3).getEntry();
  private NetworkTableEntry FFterm = tab_backClimb.add("FF Term", 0.0).withPosition(1, 4).getEntry();

  /** Creates a new backClimb. */
  public BackClimb() {
    backWinch.setIdleMode(IdleMode.kBrake);
    backWinch.setInverted(true);
    encoder = backWinch.getEncoder();
    pidController = backWinch.getPIDController();

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
      backWinch.set(speed);
    }
  }

  public void moveArm(double setPoint){
    pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  public void stopMotor(){
    backWinch.stopMotor();
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

}
