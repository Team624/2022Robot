// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
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
import frc.robot.trobot5013lib.led.ChaosPattern;
import frc.robot.trobot5013lib.led.ChasePattern;
import frc.robot.trobot5013lib.led.SolidColorPattern;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;
import frc.robot.trobot5013lib.led.TrobotAddressableLEDPattern;

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

  private DigitalInput IrSensor_tower = new DigitalInput(0);

  private ShuffleboardTab tab_tower = Shuffleboard.getTab("Tower");
  private NetworkTableEntry setSpeed_tower = tab_tower.add("Set T-Speed", false).withPosition(0, 0).getEntry();
  private NetworkTableEntry speed_tower = tab_tower.add("T-Speed", 0.0).withPosition(0, 1).getEntry();

  private NetworkTableEntry setPoint_tower = tab_tower.add("T-Setpoint", 0.0).withPosition(1, 0).getEntry();
  private NetworkTableEntry currentSpeed_tower = tab_tower.add("T-Encoder", 0.0).withPosition(1, 1).getEntry();

  private NetworkTableEntry colorDetected = tab_tower.add("Color", 0.0).withPosition(2, 0).getEntry();
  private NetworkTableEntry ballProximity = tab_tower.add("Prox", 0.0).withPosition(2, 1).getEntry();

  private NetworkTableEntry towerIR = tab_tower.add("T-IR", false).withPosition(2, 3).getEntry();
  private NetworkTableEntry feederIR = tab_tower.add("F-IR", false).withPosition(3, 3).getEntry();

  private NetworkTableEntry setSpeed_feeder = tab_tower.add("Set F-Speed", false).withPosition(4, 0).getEntry();
  private NetworkTableEntry speed_feeder = tab_tower.add("Feeder F-Speed", 0.0).withPosition(4, 1).getEntry();

  private NetworkTableEntry setPoint_feeder = tab_tower.add("F-Setpoint", 0.0).withPosition(5, 0).getEntry();
  private NetworkTableEntry currentSpeed_feeder = tab_tower.add("F-Encoder", 0.0).withPosition(5, 1).getEntry();

 

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

  private double feederPower = Constants.Feeder.feederPower;

  private ColorSensorV3 cSense;

  private final ColorMatch colorMatcher = new ColorMatch();

  // 0.229736328125 0.45654296875 0.314208984375
  // 0.330810546875 0.445556640625 0.22412109375
  private final Color kBlueTarget = new Color(0.179, 0.43, 0.39); //(.235, .47, .294)
  private final Color kRedTarget = new Color(0.50, 0.359, 0.13); //(.356, .434, .209)

  private int alliance;

  private boolean allowReverse;

  private Color[] redWhiteArray = {Color.kRed, Color.kWhiteSmoke};
  private TrobotAddressableLEDPattern m_redChasePattern = new ChasePattern(redWhiteArray, 3);
  private Color[] blueWhiteArray = {Color.kBlue, Color.kWhiteSmoke};
  private TrobotAddressableLEDPattern m_blueChasePattern = new ChasePattern(blueWhiteArray, 3);

  private TrobotAddressableLEDPattern m_greenPattern = new SolidColorPattern(Color.kGreen);
  private TrobotAddressableLEDPattern m_redPattern = new SolidColorPattern(Color.kRed);
  private TrobotAddressableLEDPattern m_bluePattern = new SolidColorPattern(Color.kBlue);
  //private TrobotAddressableLEDPattern m_shooting = new BlinkingPattern(Color.kGreen, 0.05);
  private Color[] greenWhiteArray = {Color.kGreen, Color.kSeaGreen};
  private TrobotAddressableLEDPattern m_onTarget = new ChasePattern(greenWhiteArray, 3);

  private Color[] greenRedArray = {Color.kGreen, Color.kRed};
  private TrobotAddressableLEDPattern m_shootingRed = new ChasePattern(greenRedArray, 3);

  private Color[] greenBlueArray = {Color.kGreen, Color.kBlue};
  private TrobotAddressableLEDPattern m_shootingBlue = new ChasePattern(greenBlueArray, 3);

  private TrobotAddressableLEDPattern m_climbLED = new ChaosPattern();
  
  private TrobotAddressableLED m_led;
  private double ledState = 0;
  private boolean rpmOnTarget = false;
  private boolean angleOnTarget = false;


  /** Creates a new Tower. */
  public Tower(TrobotAddressableLED m_led_strip) {
    // TOWER STUFF
    m_led = m_led_strip;

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
    feederMotor.setIdleMode(IdleMode.kCoast);

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

    allowReverse = false;
  }

  @Override
  public void periodic() {
    checkNT();
    if (ledState == 4){
      m_led.setPattern(m_climbLED);
    }
    if (!checkTowerIR()){
      SmartDashboard.getEntry("/auto/numBall").setNumber(0);
      if (alliance == 1 && ledState == 1){
        m_led.setPattern(m_bluePattern);
      } else if (ledState == 1){
        m_led.setPattern(m_redPattern);
      } else if (ledState == 2){
        m_led.setPattern(m_onTarget);
      } else if (ledState == 3 && alliance == 1){
        m_led.setPattern(m_shootingBlue);
      } else if (ledState == 3){
        m_led.setPattern(m_shootingRed);
      }
    } else if (!checkFeederIR()){
      SmartDashboard.getEntry("/auto/numBall").setNumber(1);
      if (alliance == 1 && ledState == 1){
        m_led.setPattern(m_blueChasePattern);
      } else if (ledState == 1){
        m_led.setPattern(m_redChasePattern);
      } else if (ledState == 2){
        m_led.setPattern(m_onTarget);
      } else if (ledState == 3 && alliance == 1){
        m_led.setPattern(m_shootingBlue);
      } else if (ledState == 3){
        m_led.setPattern(m_shootingRed);
      }
    } else{
      SmartDashboard.getEntry("/auto/numBall").setNumber(2);
      if (ledState == 1){
        m_led.setPattern(m_greenPattern);
      } else if (ledState == 2){
        m_led.setPattern(m_onTarget);
      } else if (ledState == 3 && alliance == 1){
        m_led.setPattern(m_shootingBlue);
      } else if (ledState == 3){
        m_led.setPattern(m_shootingRed);
      }
    }
  }

  private void checkNT(){
    // TOWER STUFF
    if(setSpeed_tower.getBoolean(false)){
      towerPower = speed_tower.getDouble(Constants.Tower.towerPower);
    }else{
      towerPower = Constants.Tower.towerPower;
    }

    currentSpeed_tower.setDouble(towerEncoder.getVelocity());
    setPoint_tower.setDouble(towerPower * Constants.Feeder.maxRPM);


    // FEEDER STUFF
    if(setSpeed_feeder.getBoolean(false)){
      feederPower = speed_feeder.getDouble(Constants.Feeder.feederPower);
    }else{
      feederPower = Constants.Feeder.feederPower;
    }

    currentSpeed_feeder.setDouble(feederEncoder.getVelocity());
    setPoint_feeder.setDouble(feederPower * Constants.Feeder.maxRPM);

    towerIR.setBoolean(IrSensor_tower.get());
    feederIR.setBoolean(IrSensor_feeder.get());
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

  // 0 is disabled, 1 is idle, 2 is on target, 3 is shooting
  public void setClimbLED(){
    ledState = 4;
  }

  public void setShootingLED(){
    ledState = 3;
  }

  public void setRpmOnTarget(boolean onTarget){
    rpmOnTarget = onTarget;
    if (rpmOnTarget && angleOnTarget){
      ledState = 2;
    }
    else{
      setIdleLED();
    }
  }

  public void setAngleOnTarget(boolean onTarget){
    angleOnTarget = onTarget;
    if (rpmOnTarget && angleOnTarget){
      ledState = 2;
    }
    else{
      setIdleLED();
    }
  }

  public boolean getRpmOnTarget(){
    return rpmOnTarget;
  }

  public boolean getAngleOnTarget(){
    return angleOnTarget;
  }

  public void setIdleLED(){
    ledState = 1;
  }

  public void setDisabledLED(){
    ledState = 0;
  }

  // FEEDER STUFF
  public void powerFeeder() {
    feederPID.setReference(feederPower * Constants.Feeder.maxRPM, CANSparkMax.ControlType.kVelocity);
    setPoint_feeder.setDouble(feederPower);
  }

  public void reverseFeeder() {
    feederPID.setReference(-(feederPower/3) * Constants.Feeder.maxRPM, CANSparkMax.ControlType.kVelocity);
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

  public int ballAlliance(){
    // nukber 1 is blue, 2 is red
    Color detectedColor = cSense.getColor();
    System.out.println("Blue: " + detectedColor.blue + " Red: " + detectedColor.red + " Green: " + detectedColor.green);
    ballProximity.setDouble(cSense.getProximity());
    int number = 0;

      if(detectedColor.red > .26){
        number = 2;
      }else if(detectedColor.red < .24){
        number = 1;
      }
    
    else{
      number = 0;
    }


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

  public void enableColorSensor(){
    allowReverse = true;
  }

  public void disableColorSensor(){
    allowReverse = false;
  }

  public boolean getReverse(){
    return allowReverse;
  }

}
