// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  CANSparkMax topShooterMotor = new CANSparkMax(20, MotorType.kBrushless);
  CANSparkMax lowShooterMotor = new CANSparkMax(23, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {
    lowShooterMotor.follow(topShooterMotor, false);
    lowShooterMotor.setSmartCurrentLimit(30);
    topShooterMotor.setSmartCurrentLimit(30);
  }

  // public Command setShooterPower(double power) {
  //   return runEnd(
  //         () -> topShooterMotor.set(power),
  //         () -> topShooterMotor.set(0)
  //       );   }

  public void setShooterPower(double power) {
    topShooterMotor.set(power);      
   }

   

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
