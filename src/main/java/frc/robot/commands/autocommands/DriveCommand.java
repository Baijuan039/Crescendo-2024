// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveCommand extends InstantCommand {
  private final DriveSubsystem m_driver;
  private final String action;

  public DriveCommand(DriveSubsystem driver, String act) {
     this.m_driver = driver;
     this.action = act;

     addRequirements(m_driver);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (action == "zero") {
      m_driver.zeroHeading();
    }
    
  }
}
