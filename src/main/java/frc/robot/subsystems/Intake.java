// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.intakeMotorID, MotorType.kBrushless);
  private Solenoid leftIntakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Intake.leftIntakeSolenoidID);
  private Solenoid rightIntakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Intake.rightIntakeSolenoidID);

  private ShuffleboardTab tab = Shuffleboard.getTab("Intake");
  private NetworkTableEntry setSpeed = tab.add("Set Speed", false).withPosition(0, 0).getEntry();
  private NetworkTableEntry intakeSpeed = tab.add("Intake Speed", 0.0).withPosition(0, 1).getEntry();

  private double intakePower = Constants.Intake.intakePower;

  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    checkNT();
    // This method will be called once per scheduler run
  }

  public void checkNT(){
    if(setSpeed.getBoolean(false)){
      intakePower = intakeSpeed.getDouble(Constants.Intake.intakePower);
    }
  }

  public void powerIntake(){
    intakeMotor.set(intakePower);
  }

  public void actuateSolenoids(){
    leftIntakeSolenoid.set(true);
    rightIntakeSolenoid.set(true);
  }

  public void retractSolenoids(){
    leftIntakeSolenoid.set(false);
    rightIntakeSolenoid.set(false);
  }

}
