// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Intake extends SubsystemBase {
  XboxController operatorXboxController;
  CANSparkMax intakeSparkMax = new CANSparkMax(21, MotorType.kBrushed);


  /** Creates a new Intake. */
  public Intake(CommandXboxController operatorJoystick) {
    this.operatorXboxController = operatorXboxController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void booleanSetSpeed(boolean bool, double speed) {
    if (bool == true) {
      intakeSparkMax.set(speed);
    }
  }

  public void testInstantCommand() {

  }

  public Command setIntakePower(double power) {
    return runEnd(
      () -> intakeSparkMax.set(power),
      () -> intakeSparkMax.set(0)

    ); 
  }


  


}
