// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private RelativeEncoder encoder;

 

  private SparkMaxPIDController intakePID;

  private Solenoid intakeSolenoid;

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

  private ShuffleboardTab tab = Shuffleboard.getTab("Intake");
  private NetworkTableEntry setSpeed = tab.add("Set Speed", false).withPosition(0, 0).getEntry();
  private NetworkTableEntry intakeSpeed = tab.add("Intake Speed", 0.0).withPosition(0, 1).getEntry();

  private NetworkTableEntry setPoint = tab.add("Setpoint", 0.0).withPosition(2, 0).getEntry();
  private NetworkTableEntry currentSpeed = tab.add("Encoder", 0.0).withPosition(2, 1).getEntry();

  private NetworkTableEntry Pterm = tab.add("P Term", 0.0).withPosition(1, 0).getEntry();
  private NetworkTableEntry Iterm = tab.add("I Term", 0.0).withPosition(1, 1).getEntry();
  private NetworkTableEntry Dterm = tab.add("D Term", 0.0).withPosition(1, 2).getEntry();
  private NetworkTableEntry IzTerm = tab.add("Iz Term", 0.0).withPosition(1, 3).getEntry();
  private NetworkTableEntry FFterm = tab.add("FF Term", 0.0).withPosition(1, 4).getEntry();

  //private NetworkTableEntry AgitateTime = tab.add("Agitate Time", Constants.Intake.agitateTime).withPosition(3, 0).getEntry();
  private NetworkTableEntry AgitateSpeed = tab.add("Agitate Speed", Constants.Intake.agitateSpeed).withPosition(3,1).getEntry();

  private double intakePower = Constants.Intake.intakePower;
  public boolean slow = false;

  /** Creates a new Intake. */
  public Intake() {
    intakeSolenoid = new Solenoid(30, PneumaticsModuleType.CTREPCM, Constants.Intake.intakeSolenoidID);
    intakeMotor = new CANSparkMax(Constants.Intake.intakeMotorID, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    encoder = intakeMotor.getEncoder();
    intakePID = intakeMotor.getPIDController();
    

    P = Constants.Intake.P;
    I = Constants.Tower.I;
    D = Constants.Tower.D;
    Iz = Constants.Tower.Iz;
    FF = Constants.Tower.FF;
    MaxOutput = Constants.Tower.MaxOutput;
    MinOutput = Constants.Tower.MinOutput;

    intakePID.setP(P);
    intakePID.setI(I);
    intakePID.setD(D);
    intakePID.setIZone(Iz);
    intakePID.setFF(FF);
    intakePID.setOutputRange(MinOutput, MaxOutput);
  }

  @Override
  public void periodic() {
    checkNT();
    // This method will be called once per scheduler run
  }

  public void checkNT(){
    if(setSpeed.getBoolean(false)){
      intakePower = intakeSpeed.getDouble(Constants.Intake.intakePower);
    } else if (slow){
      intakePower = 0.2;
    } else{
      intakePower = Constants.Intake.intakePower;
    }

    Pnew = Pterm.getDouble(Constants.Tower.P);
    Inew = Iterm.getDouble(Constants.Tower.I);
    Dnew = Dterm.getDouble(Constants.Tower.D);
    Iznew = IzTerm.getDouble(Constants.Tower.Iz);
    FFnew = FFterm.getDouble(Constants.Tower.FF);

    if((P != Pnew && Pnew != 0.0)) { intakePID.setP(Pnew); P = Pnew; }
    if((I != Inew && Inew != 0.0)) { intakePID.setI(Inew); I = Inew; }
    if((D != Dnew && Dnew != 0.0)) { intakePID.setD(Dnew); D = Dnew; }
    if((Iz != Iznew && Iznew != 0.0)) { intakePID.setIZone(Iznew); Iz = Iznew; }
    if((FF != FFnew && FFnew != 0.0)) { intakePID.setFF(FFnew); FF = FFnew; }

    currentSpeed.setDouble(encoder.getVelocity());
    setPoint.setDouble(intakePower * Constants.Intake.maxRPM);

  }

  public void powerIntake(){
    intakeMotor.set(intakePower);
  }

  public void agitate(){
    intakeMotor.set(AgitateSpeed.getDouble(Constants.Intake.agitateSpeed));
  }

  public void stopIntake(){
    intakeMotor.stopMotor();
  }

  public void actuateSolenoids(){
    intakeSolenoid.set(true);
  }

  public void retractSolenoids(){
    intakeSolenoid.set(false);
  }

}
