// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class Shoot extends CommandBase {
  private final Tower tower;

  public Shoot(Tower tower) {
    this.tower = tower;
    addRequirements(this.tower);
  }

  @Override
  public void initialize() {
    tower.setShootingLED();
  }

  @Override
  public void execute() {
    tower.powerTower();
    tower.powerFeeder();
  }

}
