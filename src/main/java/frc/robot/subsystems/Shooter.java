// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class Shooter extends SubsystemBase {

  private boolean manualShoot, manualHood, isAutoShoot, autoHood;

  private boolean ntUpdatePID, updatedPID;

  private double camDist;

  private TalonFX leftFlywheel = new TalonFX(Constants.Shooter.leftFlywheelMotorID);
  private TalonFX rightFlywheel = new TalonFX(Constants.Shooter.rightFlywheelMotorID);
  private Solenoid hood = new Solenoid(PneumaticsModuleType.REVPH, Constants.Shooter.flywheelSolenoidID);

  private boolean elToldo;//not a Hood
  private boolean isPriming;

  private double goalRPM;

  // Set up the dashboard
  private ShuffleboardTab shootTab = Shuffleboard.getTab("Shooter");
  private NetworkTableEntry dashSetRPM = shootTab.add("Goal RPM:", 0).withPosition(0, 1).getEntry();
  private NetworkTableEntry dashCurrentRPM = shootTab.add("Current RPM:", 0).withPosition(1, 1).getEntry();
  private NetworkTableEntry dashPrime = shootTab.add("Priming?", false).withPosition(2, 1).getEntry();
  private NetworkTableEntry manualRPM = shootTab.add("Manually Set RPM: ", 0).withPosition(0, 3).withWidget(BuiltInWidgets.kTextView).getEntry();
  private NetworkTableEntry manualPercent = shootTab.add("Manually Set Percent: ", 0).withPosition(0, 3).withWidget(BuiltInWidgets.kTextView).getEntry();

  private NetworkTableEntry PID_P = shootTab.add("PID P",Constants.Shooter.kP).withPosition(0,2).withWidget(BuiltInWidgets.kTextView).getEntry();
  private NetworkTableEntry PID_I = shootTab.add("PID I",Constants.Shooter.kI).withPosition(1,2).withWidget(BuiltInWidgets.kTextView).getEntry();
  private NetworkTableEntry PID_Izone = shootTab.add("PID I-zone",Constants.Shooter.kIzone).withPosition(2,2).withWidget(BuiltInWidgets.kTextView).getEntry();
  private NetworkTableEntry PID_D = shootTab.add("PID D",Constants.Shooter.kD).withPosition(3,2).withWidget(BuiltInWidgets.kTextView).getEntry();
  private NetworkTableEntry PID_F = shootTab.add("PID F",Constants.Shooter.kF).withPosition(4,2).withWidget(BuiltInWidgets.kTextView).getEntry();
  private NetworkTableEntry tuningPID = shootTab.add("Tuning PID?", false).withPosition(3,1).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

  /** Creates a new Shooter. */
  public Shooter() {
    leftFlywheel.setInverted(true);

    leftFlywheel.configFactoryDefault();
    rightFlywheel.configFactoryDefault();

    leftFlywheel.configPeakOutputForward(1);
    leftFlywheel.configPeakOutputReverse(-1);
    rightFlywheel.configPeakOutputForward(1);
    rightFlywheel.configPeakOutputReverse(-1);

    rightFlywheel.config_kP(0, Constants.Shooter.kP);
    rightFlywheel.config_kI(0, Constants.Shooter.kI);
    rightFlywheel.config_kD(0, Constants.Shooter.kD);
    rightFlywheel.config_kF(0, Constants.Shooter.kF);
    rightFlywheel.config_IntegralZone(0, Constants.Shooter.kIzone);

    leftFlywheel.config_kP(0, Constants.Shooter.kP);
    leftFlywheel.config_kI(0, Constants.Shooter.kI);
    leftFlywheel.config_kD(0, Constants.Shooter.kD);
    leftFlywheel.config_kF(0, Constants.Shooter.kF);
    leftFlywheel.config_IntegralZone(0, Constants.Shooter.kIzone);

    ntUpdatePID = false;
    updatedPID = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateDash();
    updatePID();
  }

  private double getRPM() {
    return leftFlywheel.getSelectedSensorVelocity();
  }

  public void setRPM(double rpm) {
    goalRPM = rpm;
    dashSetRPM.setNumber(rpm);
    leftFlywheel.set(TalonFXControlMode.Velocity, goalRPM);
    rightFlywheel.set(TalonFXControlMode.Velocity, -goalRPM);
  }

  public void idleFlywheel() {
    leftFlywheel.set(TalonFXControlMode.PercentOutput, Constants.Shooter.idlePercent);
    rightFlywheel.set(TalonFXControlMode.PercentOutput, -Constants.Shooter.idlePercent);
  }

  public void stopFlywheel(){
    leftFlywheel.set(TalonFXControlMode.PercentOutput, 0.0);
    rightFlywheel.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void setNTPercentage(){
    leftFlywheel.set(TalonFXControlMode.PercentOutput, manualPercent.getDouble(Constants.Shooter.manualPercent));
    rightFlywheel.set(TalonFXControlMode.PercentOutput, manualPercent.getDouble(Constants.Shooter.manualPercent));
  }

  public void setElToldo(boolean isElToldoUp) {
    elToldo = isElToldoUp;
    hood.set(elToldo);
  }

  public void setPriming(boolean priming) {
    dashPrime.setBoolean(priming);
  }

  public boolean getElToldo() {
    return elToldo;
  }

  public double getRotationAngle() {
    return SmartDashboard.getEntry("/vision/rotation_angle").getNumber(-1).doubleValue();
  }

  public double getDistanceAngle() {
    return SmartDashboard.getEntry("/vision/distance_angle").getNumber(-1).doubleValue();
  }

  private void updateDash() {
    dashSetRPM.setDouble(goalRPM);
    dashCurrentRPM.setDouble(getRPM());
    ntUpdatePID = tuningPID.getBoolean(false);
    dashPrime.setBoolean(isPriming);
  }

  private void updatePID() {
    if(ntUpdatePID) {
      //Gains newGAIN = new Gains(Pnew, Inew, Dnew, Fnew, Constants.Shooter.startGains.iZone);
      leftFlywheel.config_kP(0, PID_P.getDouble(Constants.Shooter.kP));
      leftFlywheel.config_kI(0, PID_I.getDouble(Constants.Shooter.kI));
      leftFlywheel.config_IntegralZone(0, PID_Izone.getDouble(Constants.Shooter.kIzone));
      leftFlywheel.config_kD(0, PID_D.getDouble(Constants.Shooter.kD));
      leftFlywheel.config_kF(0, PID_F.getDouble(Constants.Shooter.kF));

      rightFlywheel.config_kP(0, PID_P.getDouble(Constants.Shooter.kP));
      rightFlywheel.config_kI(0, PID_I.getDouble(Constants.Shooter.kI));
      rightFlywheel.config_IntegralZone(0, PID_Izone.getDouble(Constants.Shooter.kIzone));
      rightFlywheel.config_kD(0, PID_D.getDouble(Constants.Shooter.kD));
      rightFlywheel.config_kF(0, PID_F.getDouble(Constants.Shooter.kF));
    }
  }

  public ShuffleboardTab getTab() {
    return shootTab;
  }

  public double getManualRPM() {
    return manualRPM.getDouble(0);
  }
}