package frc.robot.subsystems.shooterSystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.resetSubsystem;

public class shooterArmSubsystem extends SubsystemBase {
    CANSparkMax armMotor = new CANSparkMax(12, MotorType.kBrushless);
    DutyCycle absoluteShooterEncoder = new DutyCycle(new DigitalInput(19));
    PIDController m_ControllerArm = new PIDController(0.6000,0,0.0008); // 0.8000, 0, 0.0900
    private double desiredPosition;
    public void positionArmShooter(double desiredPosition, boolean isShooting){
         resetSubsystem.shooterHasNode = true;
         if (isShooting){
            m_ControllerArm.setP(0.2000);
            m_ControllerArm.setI(0);
            m_ControllerArm.setD(0);
         }else{
            m_ControllerArm.setP(0.6000);
            m_ControllerArm.setI(0);
            m_ControllerArm.setD(0.0008);
         }
         this.desiredPosition = desiredPosition;
    }
    public void chainCommand(){
        if (resetSubsystem.shooterHasNode){
            new WaitCommand(.500);
            double voltage = m_ControllerArm.calculate(getCurrentPosition(), desiredPosition);
            armMotor.set(voltage);
            SmartDashboard.putNumber("absoluteEncoderPosition", getCurrentPosition());
        }
    }
    public double getCurrentPosition(){
        return absoluteShooterEncoder.getOutput()*(2*Math.PI);
    }
    @Override
    public void periodic(){
        double voltage = m_ControllerArm.calculate(getCurrentPosition(), .85);
        armMotor.set(voltage);
        SmartDashboard.putNumber("armPosition", getCurrentPosition());
    }
}