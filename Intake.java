package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    
    private final CANSparkMax intake;
    private final RelativeEncoder encoder;
    private final SparkPIDController intakePID;
    

    public Intake(int intakeID) {
        
        intake = new CANSparkMax(intakeID, MotorType.kBrushless);
        intake.restoreFactoryDefaults();

        intake.setSmartCurrentLimit(20);
        encoder = intake.getEncoder();
        intakePID = intake.getPIDController();

        intakePID.setP(IntakeConstants.kP);
        intakePID.setI(IntakeConstants.kI);
        intakePID.setD(IntakeConstants.kD);
        intakePID.setIZone(IntakeConstants.kIz);
        intakePID.setFF(IntakeConstants.kFF);
        intakePID.setOutputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);
    }
    public void runAtVelocity(double setpoint) {
        intake.setSmartCurrentLimit(20);
        intakePID.setReference(setpoint, ControlType.kVelocity);

        
    }
}
