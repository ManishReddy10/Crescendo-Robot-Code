// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;

public class Intake extends SubsystemBase {
  
  public static CANSparkMax intakeSparkMax = new CANSparkMax(21, MotorType.kBrushed);
  public static SparkAbsoluteEncoder throughBoreEncoder = intakeSparkMax.getAbsoluteEncoder();

  // intakeSparkMax.getEncoder(Type.kQuadrature, 8192);

  DigitalInput infaredReflectionBottom = new DigitalInput(2); // The infared sensor closest to ground when arm is fully down
  DigitalInput infaredReflectionTop = new DigitalInput(1); // This one is closer to shooter wheels

  private ShuffleboardTab tab = Shuffleboard.getTab("Intake Sensors");

  private GenericEntry topSensorReadount =
      tab.add("Top Infared Sensor", 0).withWidget(BuiltInWidgets.kBooleanBox)
         .getEntry();


  AddressableLED m_led = new AddressableLED(3);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
  // TrobotAddressableLED m_led = new TrobotAddressableLED(3, 0);


  /** Creates a new Intake. */
  public Intake() {
    intakeSparkMax.setSmartCurrentLimit(30);
        // PWM port 9
    // Must be a PWM header, not MXP or DIO

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_led.setLength(m_ledBuffer.getLength()); // length is 60
    
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // System.out.println("LED LENGTH: " + m_ledBuffer.getLength());

    if (infaredReflectionTop.get() == false) {
      
    
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 255, 50, 0);
   }
  } else {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 0, 100);
       }
  }
    // Set the data
    m_led.setData(m_ledBuffer);

    SmartDashboard.putBoolean("Infadred", infaredReflectionTop.get());

    

    SmartDashboard.putNumber("Through Bore Encoder Readout", throughBoreEncoder.getPosition()*360);

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

  public void autonomousIntakePower(double power) {
    intakeSparkMax.set(power);
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
