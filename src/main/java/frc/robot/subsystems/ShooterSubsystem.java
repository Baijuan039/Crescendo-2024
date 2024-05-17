// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax m_leftShooter = new CANSparkMax((Constants.OutakeConstants.kOutakeCanId1),
      MotorType.kBrushless);
  private final CANSparkMax m_rightShooter = new CANSparkMax((Constants.OutakeConstants.kOutakeCanId2),
      MotorType.kBrushless);

  private static ShooterSubsystem instance;

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  public void shootNormal() {
    m_leftShooter.set(.6);
    m_rightShooter.set(-.5);
  }

  public void shootSlow() {
    m_leftShooter.set(1);
    m_rightShooter.set(-.9);
  }

  public void frontIntake() {
    m_leftShooter.set(-.65);
    m_rightShooter.set(.75);
  }

  public void stop() {
    m_leftShooter.set(0);
    m_rightShooter.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
