package frc.robot.subsystems.shooterSystems;

import java.time.Period;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.resetSubsystem;

public class shooterRollersSubsystem extends SubsystemBase {
    CANSparkMax upwardMotorRight = new CANSparkMax(10, MotorType.kBrushless); 
    CANSparkMax upwardMotorRightAbove = new CANSparkMax(36, MotorType.kBrushless);
    CANSparkMax upwardMotorLeft = new CANSparkMax(45, MotorType.kBrushless);
    CANSparkMax shooterMotorLeft = new CANSparkMax(25, MotorType.kBrushless);
    CANSparkMax shooterMotorRight = new CANSparkMax(16, MotorType.kBrushless);
    private Timer indexTimer = new Timer();
    private boolean amp = false;
    private double shooterSpeed = 1;
    private double ampLimit = 7; // ideal 5 for perfect nodes
    public void setShooterMotors(double speed){
        shooterMotorLeft.set(speed);
        shooterMotorRight.set(-speed);
        
    }
    public void activateShooter(boolean determiner){
        if (determiner && resetSubsystem.shooterHasNode){
            setShooterMotors(1); 
        }else{
            setShooterMotors(0);
        }
    }
    public void setIndexMotors(double indexMotorSpeed, boolean amp){
        upwardMotorRight.set(indexMotorSpeed);
        upwardMotorRightAbove.set(indexMotorSpeed);
        upwardMotorLeft.set(-indexMotorSpeed);
        if (amp){
            this.amp = true;
        }
    }
    public void detectAmperage(){
        indexTimer.start();
        SmartDashboard.putNumber("timerForAmp", indexTimer.get());
        SmartDashboard.putNumber("timerForAmp", (upwardMotorLeft.getOutputCurrent() + upwardMotorRight.getOutputCurrent()));
        if ((upwardMotorLeft.getOutputCurrent() + upwardMotorRight.getOutputCurrent())  > ampLimit && indexTimer.get() > 1){
            setIndexMotors(0, false);
            RobotContainer.m_IntakeRollerSubsystem.setMotor(0);
            upwardMotorRightAbove.set(-.25);
            RobotContainer.m_ShooterArmSubsystem.positionArmShooter(1.476832584083902, false);
            amp = false;
            indexTimer.stop();
            indexTimer.reset();
        }
        
    }
    public void periodic(){
        if (amp){
            detectAmperage();
        }
    }
}