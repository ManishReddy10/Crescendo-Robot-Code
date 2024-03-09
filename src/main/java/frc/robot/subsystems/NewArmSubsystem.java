// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NewArmSubsystem extends SubsystemBase {

  CANSparkMax leftArmMotor = new CANSparkMax(24, MotorType.kBrushless);
  CANSparkMax rightArmMotor = new CANSparkMax(22, MotorType.kBrushless);
  
  RelativeEncoder leftEncoder = leftArmMotor.getEncoder();

  PIDController armPidController = new PIDController(1, 0, 0);

  DigitalInput armLimitSwitch = new DigitalInput(0);

  private ShuffleboardTab tab = Shuffleboard.getTab("New Arm Setpoints Degrees");


  /** Creates a new NewArmSubsystem. */
  public NewArmSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
