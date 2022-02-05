// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.counter.UpDownCounter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class PrimeShooter extends CommandBase {
  private final Shooter shooter;

  // TODO: Temporary variables
  private double currentAngle = 15.0;
  private double positionTwoDistance = 10;
  private double deadZone = 3;
  private double[][] experimentData = {
  //{angle, rpm}
    {5, 250}, 
    {10, 526},
    {14, 632} 
  };

  public PrimeShooter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This assumes that the data points are already sorted from smallest to largest angle
    int upperDataPoint = experimentData.length - 1;
    for (int i = 0; i < experimentData.length; i++) {
       if (currentAngle < experimentData[i][0]) {
        upperDataPoint = i;
        break;
       }
    }

    double lowerAngle = experimentData[upperDataPoint - 1][0];
    double lowerRpm = experimentData[upperDataPoint - 1][1];
    double upperAngle = experimentData[upperDataPoint][0];
    double upperRpm = experimentData[upperDataPoint][1];

    double slope = (upperRpm - lowerRpm) / (upperAngle - lowerAngle);
    
    double rpm = slope * (currentAngle - upperAngle) + upperRpm;

    // true = down; false = up
    boolean pos = currentAngle < positionTwoDistance;
    
    // TODO: Somehow move the shooter with pos and rpm variables.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
