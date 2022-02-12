// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tower extends SubsystemBase {
  private CANSparkMax towerMotor = new CANSparkMax(Constants.Tower.towerMotorID, MotorType.kBrushless);

  private DigitalInput IrSensor = new DigitalInput(0);

  private ShuffleboardTab tab = Shuffleboard.getTab("Tower");
  private NetworkTableEntry setSpeed = tab.add("Set Speed", false).withPosition(0, 0).getEntry();
  private NetworkTableEntry towerSpeed = tab.add("Tower Speed", 0.0).withPosition(0, 1).getEntry();

  private double towerPower = Constants.Tower.towerPower;
  private double autoLoadPower = Constants.Tower.autoLoadPower;

  /** Creates a new Tower. */
  public Tower() {
    towerMotor.setInverted(true);
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
  }

  public boolean checkIR(){
    return IrSensor.get();
  }

  public void powerTower(){
    towerMotor.set(towerPower);
  }

  public void loadTower(){
    towerMotor.set(autoLoadPower);
  }

  public void stopTower(){
    towerMotor.stopMotor();
  }
}
