// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_intake1 = new CANSparkMax((Constants.IntakeConstants.kIntakeCanId1),
      MotorType.kBrushless);
  private final CANSparkMax m_intake2 = new CANSparkMax((Constants.IntakeConstants.kIntakeCanId2),
      MotorType.kBrushless);
  private final CANSparkMax m_intake3 = new CANSparkMax((Constants.IntakeConstants.kIntakeCanId3),
      MotorType.kBrushed);

  private static IntakeSubsystem instance;

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  public void mainIntake() {
    m_intake1.set(-.45);
    m_intake2.set(.65);
    m_intake3.set(.35);
  }

  public void mainOutake() {
    m_intake1.set(0.3);
    m_intake2.set(-0.5);
    m_intake3.set(-.5);
  }

  public void stop() {
    m_intake1.set(0);
    m_intake2.set(0);
    m_intake3.set(0);
  }

  public void tertiary() {
    m_intake3.set(-0.5);
  }

  // Used when Shooting
  public void tertiaryDelay() {
    Timer.delay(2);
    m_intake3.set(1);
  }

  // Used when Shooting fast
  public void tertiaryFastDelay() {
    Timer.delay(1);
    m_intake3.set(1);
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
