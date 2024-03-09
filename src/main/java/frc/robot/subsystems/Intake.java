// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Intake extends SubsystemBase {
  XboxController operatorXboxController;
  
  CANSparkMax intakeSparkMax = new CANSparkMax(21, MotorType.kBrushed);

  DigitalInput infaredReflectionBottom = new DigitalInput(2); // The infared sensor closest to ground when arm is fully down
  DigitalInput infaredReflectionTop = new DigitalInput(1); // This one is closer to shooter wheels

  private ShuffleboardTab tab = Shuffleboard.getTab("Intake Sensors");

  private GenericEntry topSensorReadount =
      tab.add("Top Infared Sensor", 0).withWidget(BuiltInWidgets.kBooleanBox)
         .getEntry();




  /** Creates a new Intake. */
  public Intake(CommandXboxController operatorJoystick) {
    this.operatorXboxController = operatorXboxController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    topSensorReadount.setBoolean(infaredReflectionTop.get());
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

    public Command runIntakeUntilBeamBreakCommand() {
    return runEnd(
      () -> runIntakeUntilBeamBreak(),
      () -> intakeSparkMax.set(0)

    ); 
  }

  public void runIntakeUntilBeamBreak() {
    // if (infaredReflectionTop.get() && infaredReflectionBottom.get()) {
    if (infaredReflectionTop.get() == false) {
      intakeSparkMax.set(0);
    } else {
      intakeSparkMax.set(0.6);
    }
  }


  


}
