// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tower extends SubsystemBase {

  // TOWER STUFF
  private CANSparkMax towerMotor;
  private RelativeEncoder towerEncoder;

  private SparkMaxPIDController towerPID;

  private double P_tower;
  private double I_tower; 
  private double D_tower; 
  private double Iz_tower; 
  private double FF_tower;
  private double MaxOutput_tower;
  private double MinOutput_tower; 

  private double Pnew_tower;
  private double Inew_tower;
  private double Dnew_tower;
  private double Iznew_tower;
  private double FFnew_tower;

  private DigitalInput IrSensor_tower = new DigitalInput(0);

  private ShuffleboardTab tab_tower = Shuffleboard.getTab("Tower");
  private NetworkTableEntry setSpeed_tower = tab_tower.add("Set Speed", false).withPosition(0, 0).getEntry();
  private NetworkTableEntry speed_tower = tab_tower.add("Tower Speed", 0.0).withPosition(0, 1).getEntry();

  private NetworkTableEntry setPoint_tower = tab_tower.add("Setpoint", 0.0).withPosition(2, 0).getEntry();
  private NetworkTableEntry currentSpeed_tower = tab_tower.add("Encoder", 0.0).withPosition(2, 1).getEntry();

  private NetworkTableEntry Pterm_tower = tab_tower.add("P Term", 0.0).withPosition(1, 0).getEntry();
  private NetworkTableEntry Iterm_tower = tab_tower.add("I Term", 0.0).withPosition(1, 1).getEntry();
  private NetworkTableEntry Dterm_tower = tab_tower.add("D Term", 0.0).withPosition(1, 2).getEntry();
  private NetworkTableEntry IzTerm_tower = tab_tower.add("Iz Term", 0.0).withPosition(1, 3).getEntry();
  private NetworkTableEntry FFterm_tower = tab_tower.add("FF Term", 0.0).withPosition(1, 4).getEntry();

  private NetworkTableEntry colorDetected = tab_tower.add("Color", 0.0).withPosition(3, 0).getEntry();

  private double towerPower = Constants.Tower.towerPower;

  // FEEDER STUFF
  private CANSparkMax feederMotor;
  private RelativeEncoder feederEncoder;

  private SparkMaxPIDController feederPID;

  private DigitalInput IrSensor_feeder = new DigitalInput(1);

  private double P_feeder;
  private double I_feeder; 
  private double D_feeder; 
  private double Iz_feeder; 
  private double FF_feeder;
  private double MaxOutput_feeder;
  private double MinOutput_feeder; 

  private double Pnew_feeder;
  private double Inew_feeder;
  private double Dnew_feeder;
  private double Iznew_feeder;
  private double FFnew_feeder;

  private ShuffleboardTab tab_feeder = Shuffleboard.getTab("Feeder");
  private NetworkTableEntry setSpeed_feeder = tab_feeder.add("Set Speed", false).withPosition(0, 0).getEntry();
  private NetworkTableEntry speed_feeder = tab_feeder.add("Feeder Speed", 0.0).withPosition(0, 1).getEntry();

  private NetworkTableEntry setPoint_feeder = tab_feeder.add("Setpoint", 0.0).withPosition(2, 0).getEntry();
  private NetworkTableEntry currentSpeed_feeder = tab_feeder.add("Encoder", 0.0).withPosition(2, 1).getEntry();

  private NetworkTableEntry Pterm_feeder = tab_feeder.add("P Term", 0.0).withPosition(1, 0).getEntry();
  private NetworkTableEntry Iterm_feeder = tab_feeder.add("I Term", 0.0).withPosition(1, 1).getEntry();
  private NetworkTableEntry Dterm_feeder = tab_feeder.add("D Term", 0.0).withPosition(1, 2).getEntry();
  private NetworkTableEntry IzTerm_feeder = tab_feeder.add("Iz Term", 0.0).withPosition(1, 3).getEntry();
  private NetworkTableEntry FFterm_feeder = tab_feeder.add("FF Term", 0.0).withPosition(1, 4).getEntry();

  private double feederPower = Constants.Feeder.feederPower;

  private ColorSensorV3 cSense;

  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.2055, 0.45141, 0.34326); //(.235, .47, .294)
  private final Color kRedTarget = new Color(0.33715, 0.4353, 0.22875); //(.356, .434, .209)

  private int alliance;

  private boolean allowReverse;

  private AddressableLED towerLEDRight;

  private AddressableLED towerLEDLeft;

  private AddressableLEDBuffer m_ledBufferRight;

  private AddressableLEDBuffer m_ledBufferLeft;

  /** Creates a new Tower. */
  public Tower() {
    // TOWER STUFF
    towerMotor = new CANSparkMax(Constants.Tower.towerMotorID, MotorType.kBrushless);
    towerMotor.restoreFactoryDefaults();
    towerMotor.setIdleMode(IdleMode.kBrake);
    towerMotor.setInverted(true);
    towerEncoder = towerMotor.getEncoder();
    towerPID = towerMotor.getPIDController();
    cSense = new ColorSensorV3(Port.kOnboard);

    P_tower = Constants.Tower.P;
    I_tower = Constants.Tower.I;
    D_tower = Constants.Tower.D;
    Iz_tower = Constants.Tower.Iz;
    FF_tower = Constants.Tower.FF;
    MaxOutput_tower = Constants.Tower.MaxOutput;
    MinOutput_tower = Constants.Tower.MinOutput;

    towerPID.setP(P_tower);
    towerPID.setI(I_tower);
    towerPID.setD(D_tower);
    towerPID.setIZone(Iz_tower);
    towerPID.setFF(FF_tower);
    towerPID.setOutputRange(MinOutput_tower, MaxOutput_tower);


    // FEEDER STUFF
    feederMotor = new CANSparkMax(Constants.Feeder.feederMotorID, MotorType.kBrushless);
    
    feederEncoder = feederMotor.getEncoder();
    feederPID = feederMotor.getPIDController();

    feederMotor.restoreFactoryDefaults();
    feederMotor.setIdleMode(IdleMode.kBrake);

    P_feeder = Constants.Feeder.P;
    I_feeder = Constants.Feeder.I;
    D_feeder = Constants.Feeder.D;
    Iz_feeder = Constants.Feeder.Iz;
    FF_feeder = Constants.Feeder.FF;
    MaxOutput_feeder = Constants.Feeder.MaxOutput;
    MinOutput_feeder = Constants.Feeder.MinOutput;

    feederPID.setP(P_feeder);
    feederPID.setI(I_feeder);
    feederPID.setD(D_feeder);
    feederPID.setIZone(Iz_feeder);
    feederPID.setFF(FF_feeder);
    feederPID.setOutputRange(MinOutput_feeder, MaxOutput_feeder);

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kRedTarget);

    allowReverse = true;

    //9 was right

    //towerLEDRight = new AddressableLED(9);
    //towerLED2 = new AddressableLED(9);

    m_ledBufferRight = new AddressableLEDBuffer(15);
    //m_ledBuffer2 = new AddressableLEDBuffer(60);

    towerLEDRight.setLength(m_ledBufferRight.getLength());
    //towerLED2.setLength(m_ledBuffer2.getLength());

    towerLEDRight.setData(m_ledBufferRight);
    //towerLED2.setData(m_ledBuffer2);
    towerLEDRight.start();
    //towerLED2.start();
  }

  @Override
  public void periodic() {
    checkNT();
    setLED();
    if (!checkTowerIR()){
      SmartDashboard.getEntry("/auto/numBall").setNumber(0);
    } else if (!checkFeederIR()){
      SmartDashboard.getEntry("/auto/numBall").setNumber(1);
    } else{
      SmartDashboard.getEntry("/auto/numBall").setNumber(2);
    }
  }

  private void checkNT(){
    // TOWER STUFF
    if(setSpeed_tower.getBoolean(false)){
      towerPower = speed_tower.getDouble(Constants.Tower.towerPower);
    }else{
      towerPower = Constants.Tower.towerPower;
    }

    Pnew_tower = Pterm_tower.getDouble(Constants.Tower.P);
    Inew_tower = Iterm_tower.getDouble(Constants.Tower.I);
    Dnew_tower = Dterm_tower.getDouble(Constants.Tower.D);
    Iznew_tower = IzTerm_tower.getDouble(Constants.Tower.Iz);
    FFnew_tower = FFterm_tower.getDouble(Constants.Tower.FF);

    if((P_tower != Pnew_tower && Pnew_tower != 0.0)) { towerPID.setP(Pnew_tower); P_tower = Pnew_tower; }
    if((I_tower != Inew_tower && Inew_tower != 0.0)) { towerPID.setI(Inew_tower); I_tower = Inew_tower; }
    if((D_tower != Dnew_tower && Dnew_tower != 0.0)) { towerPID.setD(Dnew_tower); D_tower = Dnew_tower; }
    if((Iz_tower != Iznew_tower && Iznew_tower != 0.0)) { towerPID.setIZone(Iznew_tower); Iz_tower = Iznew_tower; }
    if((FF_tower != FFnew_tower && FFnew_tower != 0.0)) { towerPID.setFF(FFnew_tower); FF_tower = FFnew_tower; }

    currentSpeed_tower.setDouble(towerEncoder.getVelocity());
    setPoint_tower.setDouble(towerPower * Constants.Feeder.maxRPM);


    // FEEDER STUFF
    if(setSpeed_feeder.getBoolean(false)){
      feederPower = speed_feeder.getDouble(Constants.Feeder.feederPower);
    }else{
      feederPower = Constants.Feeder.feederPower;
    }

    Pnew_feeder = Pterm_feeder.getDouble(Constants.Feeder.P);
    Inew_feeder = Iterm_feeder.getDouble(Constants.Feeder.I);
    Dnew_feeder = Dterm_feeder.getDouble(Constants.Feeder.D);
    Iznew_feeder = IzTerm_feeder.getDouble(Constants.Feeder.Iz);
    FFnew_feeder = FFterm_feeder.getDouble(Constants.Feeder.FF);

    if((P_feeder != Pnew_feeder && Pnew_feeder != 0.0)) { feederPID.setP(Pnew_feeder); P_feeder = Pnew_feeder; }
    if((I_feeder != Inew_feeder && Inew_feeder != 0.0)) { feederPID.setI(Inew_feeder); I_feeder = Inew_feeder; }
    if((D_feeder != Dnew_feeder && Dnew_feeder != 0.0)) { feederPID.setD(Dnew_feeder); D_feeder = Dnew_feeder; }
    if((Iz_feeder != Iznew_feeder && Iznew_feeder != 0.0)) { feederPID.setIZone(Iznew_feeder); Iz_feeder = Iznew_feeder; }
    if((FF_feeder != FFnew_feeder && FFnew_feeder != 0.0)) { feederPID.setFF(FFnew_feeder); FF_feeder = FFnew_feeder; }

    currentSpeed_feeder.setDouble(feederEncoder.getVelocity());
    setPoint_feeder.setDouble(feederPower * Constants.Feeder.maxRPM);
  }



  // TOWER STUFF
  public boolean checkTowerIR(){
    return IrSensor_tower.get();
  }

  public void powerTower(){
    towerPID.setReference(towerPower * Constants.Tower.maxRPM, CANSparkMax.ControlType.kVelocity);
    setPoint_tower.setDouble(towerPower);
  }

  public void reverseTower(){
    towerPID.setReference(-towerPower * Constants.Tower.maxRPM, CANSparkMax.ControlType.kVelocity);
    setPoint_tower.setDouble(-towerPower);
  }

  public void manualTower(double speed){
    towerPID.setReference(speed * Constants.Tower.maxRPM, CANSparkMax.ControlType.kVelocity);
    setPoint_tower.setDouble(speed);
  }

  public void stopTower(){
    towerMotor.stopMotor();
  }

  // FEEDER STUFF
  public void powerFeeder() {
    feederPID.setReference(feederPower * Constants.Feeder.maxRPM, CANSparkMax.ControlType.kVelocity);
    setPoint_feeder.setDouble(feederPower);
  }

  public void reverseFeeder() {
    feederPID.setReference(-feederPower * Constants.Feeder.maxRPM, CANSparkMax.ControlType.kVelocity);
    setPoint_feeder.setDouble(-feederPower);
  }

  public void manualFeeder(double speed){
    feederPID.setReference(speed * Constants.Feeder.maxRPM, CANSparkMax.ControlType.kVelocity);
    setPoint_feeder.setDouble(speed);
  }

  public void stopFeeder() {
    feederMotor.stopMotor();
  }

  public boolean checkFeederIR(){
    return IrSensor_feeder.get();
  }

  public int checkAlliance(){
    // nukber 1 is blue, 2 is red
    Color detectedColor = cSense.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    int number = 0;
    if (cSense.getProximity() > 100){
      if(match.color == kBlueTarget){
        number = 1;
      }else if(match.color == kRedTarget){
        number = 2;
      }
    }
    else{
      number = 0;
    }

    //System.out.println(number);

    colorDetected.setNumber(number);
    return number;
  }

  public void updateAlliance(){
    if(DriverStation.getAlliance() == Alliance.Blue){
      alliance = 1;
    }else{
      alliance = 2;
    }
  }

  public int getAlliance(){
    return alliance;
  }

  public void setReverse(){
    if(allowReverse){
      allowReverse = false;
    }else{
      allowReverse = true;
    }
  }

  public boolean getReverse(){
    return allowReverse;
  }

  private void setLED(){
    // if(checkTowerIR() && checkFeederIR()){
    //   for (var i = 0; i < m_ledBuffer1.getLength(); i++) {
    //     m_ledBuffer1.setRGB(i, 0, 244, 0);
    //     m_ledBuffer2.setRGB(i, 0, 244, 0);
    //   }
    // }else if(checkTowerIR() == true && checkFeederIR() == false){
    //   for (var i = 0; i < m_ledBuffer1.getLength(); i++) {
    //     m_ledBuffer1.setRGB(i, 148, 255, 141);
    //     m_ledBuffer2.setRGB(i, 148, 255, 141);
    //   }
    // }else{
    //   for (var i = 0; i < m_ledBuffer1.getLength(); i++) {
    //     if(DriverStation.getAlliance() == Alliance.Red){
    //       m_ledBuffer1.setRGB(i, 255, 0, 0);
    //       m_ledBuffer2.setRGB(i, 255, 0, 0);
    //     }else{
    //       m_ledBuffer1.setRGB(i, 0, 0, 255);
    //       m_ledBuffer2.setRGB(i, 0, 0, 255);
    //     }
    //   }
    //}
    // m_ledBufferRight.setRGB(1, 0, 50, 150);
    // m_ledBufferRight.setRGB(2, 0, 40, 150);
    // m_ledBufferRight.setRGB(3, 0, 30, 150);
    // m_ledBufferRight.setRGB(4, 0, 20, 150);
    // towerLEDRight.setData(m_ledBufferRight);
    //towerLED2.setData(m_ledBuffer2);
  }

}
