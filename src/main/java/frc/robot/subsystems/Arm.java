// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  
  CANSparkMax leftArmMotor = new CANSparkMax(24, MotorType.kBrushless);
  CANSparkMax rightArmMotor = new CANSparkMax(22, MotorType.kBrushless);
  
  RelativeEncoder leftEncoder = leftArmMotor.getEncoder();

  PIDController armPidController = new PIDController(0.01, 0.001, 0);

  DigitalInput armLimitSwitch = new DigitalInput(0);

  String setPoint;


  /*Shuffleboard Initializations */
  private ShuffleboardTab tab = Shuffleboard.getTab("Arm Setpoints Degrees");

  private GenericEntry inputAmpSetpoint = tab.add("Amp Setpoint", 20).
      withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 100))
      .withSize(4, 1).getEntry();

  private GenericEntry encoderDegreesReadout =

      tab.add("Encoder Degrees Readout", 0)
         .getEntry();


  /** Creates a new Arm. */
  public Arm() {
    rightArmMotor.follow(leftArmMotor, false);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    armPidController.setSetpoint(0);
    leftEncoder.setPosition(0);
  }


  public void setArmPosition(double setpoint) {

    armPidController.setSetpoint(-setpoint);  
  }

  public void setPickupPosition() {
    // if (getPositionDegrees() > 0){
    //   armPidController.setPID(0.01, 0.001, 0.0);
    // } else {
    //   armPidController.setPID(0.3, 0.0, 0.0);
    // }
    if (setPoint.equals("Transition")) {
      armPidController.setSetpoint(0);
    }
  }

  public void setTransitionalPosition() {
    // armPidController.setPID(0.1, 0, 0);
    // if (getPositionDegrees() > 10){
    //   armPidController.setPID(0.01, 0.001, 0.0);
    // } else {
    //   armPidController.setPID(0.3, 0.0, 0.0);
    // }
    armPidController.setSetpoint(-10);
    setPoint = "Transition";
  }
  
  public void setTopPosition() {
    // if (getPositionDegrees() > 40){
    //   armPidController.setPID(0.01, 0.001, 0.0);
    // } else {
    //   armPidController.setPID(0.3, 0.0, 0.0);
    // }
    // armPidController.setPID(0.5, 0, 0);
    armPidController.setSetpoint(-40);  
    setPoint = "Top";
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
      encoderDegreesReadout.setDouble(getPositionDegrees());
      System.out.println(getPositionDegrees());

      /* Limits Arm Maximum Angle, if maximum arm angle is reached will set arm speed to 0 */
      if(getPositionDegrees() >= 80){
        leftArmMotor.set(0);
      } else {
        
        leftArmMotor.set(armPidController.calculate(getPositionDegrees()));
      }

      /*Resets neo encoder position to 0 when arm hits limit switch */
      // if(armLimitSwitch.get()) {
      //   leftEncoder.setPosition(0);
      // }

      // if (armLimitSwitch.get()) {
      //   System.out.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");        
      // }

      

    }
  }
