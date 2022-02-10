// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tower extends SubsystemBase {
  private CANSparkMax towerMotor = new CANSparkMax(Constants.Tower.towerMotorID, MotorType.kBrushless);

  private ShuffleboardTab tab = Shuffleboard.getTab("Tower");
  private NetworkTableEntry setSpeed = tab.add("Set Speed", false).withPosition(0, 0).getEntry();
  private NetworkTableEntry towerSpeed = tab.add("Tower Speed", 0.0).withPosition(0, 1).getEntry();

  private double towerPower = Constants.Tower.towerPower;

  /** Creates a new Tower. */
  public Tower() {}

  @Override
  public void periodic() {
    checkNT();
  }

  private void checkNT(){
    if(setSpeed.getBoolean(false)){
      towerPower = towerSpeed.getDouble(Constants.Tower.towerPower);
    }
  }

  public double checkIR(){
    return 0.0;
  }

  public void powerTower(){
    towerMotor.set(towerPower);
  }
}
