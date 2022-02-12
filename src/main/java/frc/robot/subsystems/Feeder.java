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

public class Feeder extends SubsystemBase {
  private CANSparkMax feederMotor = new CANSparkMax(Constants.Feeder.feederMotorID, MotorType.kBrushless);

  private ShuffleboardTab tab = Shuffleboard.getTab("Feeder");
  private NetworkTableEntry setSpeed = tab.add("Set Speed", false).withPosition(0, 0).getEntry();
  private NetworkTableEntry feederSpeed = tab.add("Feeder Speed", 0.0).withPosition(0, 1).getEntry();

  private double feederPower = Constants.Feeder.feederPower;

  /** Creates a new Hopper. */
  public Feeder() {}

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
  }

  public void powerFeeder() {
    feederMotor.set(feederPower);
  }

  public void stopFeeder() {
    feederMotor.stopMotor();
  }
}
