package frc.robot.subsystems.intakeSystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.resetSubsystem;

public class intakeRollerSubsystem extends SubsystemBase {
    CANSparkMax rightRollerMotor = new CANSparkMax(15, MotorType.kBrushless);
    CANSparkMax leftRollerMotor = new CANSparkMax(14, MotorType.kBrushless);
    Timer rollerTimer = new Timer();
    Timer finalTimer = new Timer();
    private double speed = 1;
    private double ampLimited = 10; // 
    public void setMotor(double speed){
        rightRollerMotor.set(speed);
        leftRollerMotor.set(speed);
    }
    public void activateRollers(boolean determiner){
        resetSubsystem.isHome = false; // made 10:09
        if (determiner){
            setMotor(speed);
            checkAmperage();
        }else{
            setMotor(0);
        }
    }
    public void checkAmperage(){
        rollerTimer.start();
        while (true){
            SmartDashboard.putBoolean("isReady", !resetSubsystem.intakeHasNode);
            SmartDashboard.putBoolean("gotRan", false);
            if (leftRollerMotor.getOutputCurrent() >= ampLimited && rollerTimer.get() > .25 && !resetSubsystem.intakeHasNode){
                SmartDashboard.putBoolean("gotRan", true);
                RobotContainer.m_ShooterArmSubsystem.positionArmShooter(0.988694418639788, false);
                setMotor(0);
                RobotContainer.m_AmpSpin.slowRotation(true);
                resetSubsystem.intakeHasNode = true;
                // 
                setMotor(0);
                rollerTimer.stop();
                rollerTimer.reset();
                // resetSubsystem.shooterHasNode = false;
                RobotContainer.m_ShooterRollersSubsystem.setIndexMotors(1,true);
                RobotContainer.m_ShooterRollersSubsystem.detectAmperage();
                break;
            }
        }
    }

}