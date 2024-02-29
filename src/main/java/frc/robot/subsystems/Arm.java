// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  
  CANSparkMax leftArmMotor = new CANSparkMax(24, MotorType.kBrushless);
  CANSparkMax rightArmMotor = new CANSparkMax(22, MotorType.kBrushless);
  
  RelativeEncoder leftEncoder = leftArmMotor.getEncoder();

  PIDController armPidController = new PIDController(0.2, 0, 0);

  /** Creates a new Arm. */
  public Arm() {
    rightArmMotor.follow(leftArmMotor, true);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    armPidController.setSetpoint(0);
  }

  public void setAmpPosition(double setpoint) {
    armPidController.setSetpoint(setpoint);
  }
  
  
  public Command setArmPower(double power){
    return runEnd(
      () -> leftArmMotor.set(power),
      () -> leftArmMotor.set(0)
      
      ); 
    }
    
    /*Converts Neo internal motor encoder to degrees */
    public double getPositionDegrees() {
      return (Units.rotationsToDegrees(leftEncoder.getPosition())/256);
    }
    

    @Override
    public void periodic() {
      
      /*This method will be called once per scheduler run  */
      SmartDashboard.putNumber("arm position", getPositionDegrees());

      /* Limits Arm Maximum Angle, if maximum arm angle is reached will set arm speed to 0 */
      if(getPositionDegrees() >= 80){
        leftArmMotor.set(0);
      } else {
        
        leftArmMotor.set(armPidController.calculate(getPositionDegrees()));
      }


    }
  }
