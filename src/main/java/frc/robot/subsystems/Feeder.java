// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private CANSparkMax feederMotor;
  private RelativeEncoder encoder;

  private SparkMaxPIDController feederPID;

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

  private ShuffleboardTab tab = Shuffleboard.getTab("Feeder");
  private NetworkTableEntry setSpeed = tab.add("Set Speed", false).withPosition(0, 0).getEntry();
  private NetworkTableEntry feederSpeed = tab.add("Feeder Speed", 0.0).withPosition(0, 1).getEntry();

  private NetworkTableEntry setPoint = tab.add("Setpoint", 0.0).withPosition(2, 0).getEntry();
  private NetworkTableEntry currentSpeed = tab.add("Encoder", 0.0).withPosition(2, 1).getEntry();

  private NetworkTableEntry Pterm = tab.add("P Term", 0.0).withPosition(1, 0).getEntry();
  private NetworkTableEntry Iterm = tab.add("I Term", 0.0).withPosition(1, 1).getEntry();
  private NetworkTableEntry Dterm = tab.add("D Term", 0.0).withPosition(1, 2).getEntry();
  private NetworkTableEntry IzTerm = tab.add("Iz Term", 0.0).withPosition(1, 3).getEntry();
  private NetworkTableEntry FFterm = tab.add("FF Term", 0.0).withPosition(1, 4).getEntry();

  private double feederPower = Constants.Feeder.feederPower;

  /** Creates a new Hopper. */
  public Feeder() {
    feederMotor = new CANSparkMax(Constants.Feeder.feederMotorID, MotorType.kBrushless);
    encoder = feederMotor.getEncoder();
    feederPID = feederMotor.getPIDController();

    feederMotor.restoreFactoryDefaults();

    P = Constants.Feeder.P;
    I = Constants.Feeder.I;
    D = Constants.Feeder.D;
    Iz = Constants.Feeder.Iz;
    FF = Constants.Feeder.FF;
    MaxOutput = Constants.Feeder.MaxOutput;
    MinOutput = Constants.Feeder.MinOutput;

    feederPID.setP(P);
    feederPID.setI(I);
    feederPID.setD(D);
    feederPID.setIZone(Iz);
    feederPID.setFF(FF);
    feederPID.setOutputRange(MinOutput, MaxOutput);
  }

  @Override
  public void periodic() {
    checkNT();
    // This method will be called once per scheduler run
  }

  private void checkNT(){
    if(setSpeed.getBoolean(false)){
      feederPower = feederSpeed.getDouble(Constants.Feeder.feederPower);
    }else{
      feederPower = Constants.Feeder.feederPower;
    }

    Pnew = Pterm.getDouble(Constants.Feeder.P);
    Inew = Iterm.getDouble(Constants.Feeder.I);
    Dnew = Dterm.getDouble(Constants.Feeder.D);
    Iznew = IzTerm.getDouble(Constants.Feeder.Iz);
    FFnew = FFterm.getDouble(Constants.Feeder.FF);

    if((P != Pnew && Pnew != 0.0)) { feederPID.setP(Pnew); P = Pnew; }
    if((I != Inew && Inew != 0.0)) { feederPID.setI(Inew); I = Inew; }
    if((D != Dnew && Dnew != 0.0)) { feederPID.setD(Dnew); D = Dnew; }
    if((Iz != Iznew && Iznew != 0.0)) { feederPID.setIZone(Iznew); Iz = Iznew; }
    if((FF != FFnew && FFnew != 0.0)) { feederPID.setFF(FFnew); FF = FFnew; }

    currentSpeed.setDouble(encoder.getVelocity());
    System.out.println(feederPower + " " + Constants.Feeder.maxRPM);
    setPoint.setDouble(feederPower * Constants.Feeder.maxRPM);
  }

  public void powerFeeder() {
    feederPID.setReference(feederPower * Constants.Feeder.maxRPM, CANSparkMax.ControlType.kVelocity);
    setPoint.setDouble(feederPower);
    
  }

  public void stopFeeder() {
    feederMotor.stopMotor();
  }
}
