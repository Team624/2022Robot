// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tower extends SubsystemBase {
  private CANSparkMax towerMotor;
  private RelativeEncoder encoder;

  private SparkMaxPIDController towerPID;

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

  private DigitalInput TowerSensor = new DigitalInput(0);

  private ShuffleboardTab tab = Shuffleboard.getTab("Tower");
  private NetworkTableEntry setSpeed = tab.add("Set Speed", false).withPosition(0, 0).getEntry();
  private NetworkTableEntry towerSpeed = tab.add("Tower Speed", 0.0).withPosition(0, 1).getEntry();

  private NetworkTableEntry setPoint = tab.add("Setpoint", 0.0).withPosition(2, 0).getEntry();
  private NetworkTableEntry currentSpeed = tab.add("Encoder", 0.0).withPosition(2, 1).getEntry();

  private NetworkTableEntry Pterm = tab.add("P Term", 0.0).withPosition(1, 0).getEntry();
  private NetworkTableEntry Iterm = tab.add("I Term", 0.0).withPosition(1, 1).getEntry();
  private NetworkTableEntry Dterm = tab.add("D Term", 0.0).withPosition(1, 2).getEntry();
  private NetworkTableEntry IzTerm = tab.add("Iz Term", 0.0).withPosition(1, 3).getEntry();
  private NetworkTableEntry FFterm = tab.add("FF Term", 0.0).withPosition(1, 4).getEntry();

  private double towerPower = Constants.Tower.towerPower;

  /** Creates a new Tower. */
  public Tower() {
    towerMotor = new CANSparkMax(Constants.Tower.towerMotorID, MotorType.kBrushless);
    towerMotor.restoreFactoryDefaults();
    towerMotor.setInverted(true);
    encoder = towerMotor.getEncoder();
    towerPID = towerMotor.getPIDController();

    P = Constants.Tower.P;
    I = Constants.Tower.I;
    D = Constants.Tower.D;
    Iz = Constants.Tower.Iz;
    FF = Constants.Tower.FF;
    MaxOutput = Constants.Tower.MaxOutput;
    MinOutput = Constants.Tower.MinOutput;

    towerPID.setP(P);
    towerPID.setI(I);
    towerPID.setD(D);
    towerPID.setIZone(Iz);
    towerPID.setFF(FF);
    towerPID.setOutputRange(MinOutput, MaxOutput);
  }

  @Override
  public void periodic() {
    checkNT();
  }

  private void checkNT(){
    if(setSpeed.getBoolean(false)){
      towerPower = towerSpeed.getDouble(Constants.Tower.towerPower);
    }else{
      towerPower = Constants.Tower.towerPower;
    }

    Pnew = Pterm.getDouble(Constants.Tower.P);
    Inew = Iterm.getDouble(Constants.Tower.I);
    Dnew = Dterm.getDouble(Constants.Tower.D);
    Iznew = IzTerm.getDouble(Constants.Tower.Iz);
    FFnew = FFterm.getDouble(Constants.Tower.FF);

    if((P != Pnew && Pnew != 0.0)) { towerPID.setP(Pnew); P = Pnew; }
    if((I != Inew && Inew != 0.0)) { towerPID.setI(Inew); I = Inew; }
    if((D != Dnew && Dnew != 0.0)) { towerPID.setD(Dnew); D = Dnew; }
    if((Iz != Iznew && Iznew != 0.0)) { towerPID.setIZone(Iznew); Iz = Iznew; }
    if((FF != FFnew && FFnew != 0.0)) { towerPID.setFF(FFnew); FF = FFnew; }

    currentSpeed.setDouble(encoder.getVelocity());
    setPoint.setDouble(towerPower * Constants.Feeder.maxRPM);
  }

  public boolean checkIR(){
    return TowerSensor.get();
  }

  public void powerTower(){
    towerPID.setReference(towerPower * Constants.Tower.maxRPM, CANSparkMax.ControlType.kVelocity);
    setPoint.setDouble(towerPower);
  }

  public void loadTower(){
    if(!checkIR()){
      towerPID.setReference(towerPower * Constants.Tower.maxRPM, CANSparkMax.ControlType.kVelocity);
      setPoint.setDouble(towerPower);
    }else{
      towerPID.setReference(0, CANSparkMax.ControlType.kVelocity);
      setPoint.setDouble(0);
    }
  }

  public void stopTower(){
    towerMotor.stopMotor();
  }

}
