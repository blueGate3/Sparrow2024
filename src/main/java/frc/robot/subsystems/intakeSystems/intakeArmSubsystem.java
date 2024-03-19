package frc.robot.subsystems.intakeSystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.resetSubsystem;

public class intakeArmSubsystem extends SubsystemBase {
    CANSparkMax motorArmLeft = new CANSparkMax(11, MotorType.kBrushless);
    CANSparkMax motorArmRight = new CANSparkMax(13, MotorType.kBrushless);
    DutyCycle absoluteEncoder = new DutyCycle(new DigitalInput(11));
    RelativeEncoder motorRelative = motorArmLeft.getEncoder();
    // PIDController m_PidController = new PIDController(0.0800, 0.0000005, 0.0060);
    SparkPIDController m_PidController = motorArmLeft.getPIDController();
    private double desiredPosition;
    public intakeArmSubsystem(){
        m_PidController.setP(0.0800);
        m_PidController.setI(0);
        m_PidController.setD(0.0060);
        motorRelative.setPosition(0);
        // calibrateCurrent();
    }
    public void setIntakeArmPosition(double desiredPosition){
        this.desiredPosition = desiredPosition;
        // while (true){
            // new WaitCommand(.20);
            // double currentPosition = absoluteEncoder.getOutput()*(2*Math.PI);
            // double voltage = m_PidController.calculate(currentPosition, desiredPosition);
            // motorArmLeft.set(-voltage);
            m_PidController.setReference(desiredPosition, ControlType.kPosition);
        // }
    }
    public void calibrateCurrent(){
        // motorRelative.setPosition(absoluteEncoder.getOutput()*(2*Math.PI));
        // System.out.println("Updated");
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("EncoderVal", motorRelative.getPosition());
        if (resetSubsystem.intakeHasNode){
                m_PidController.setReference(0, ControlType.kPosition);
                resetSubsystem.intakeHasNode = false;
            }
            m_PidController.setReference(0, ControlType.kPosition);
            resetSubsystem.intakeHasNode = false;
        // SmartDashboard.putNumber("AbsEncoderVal", absoluteEncoder.getOutput()*(2*Math.PI));
    }

}