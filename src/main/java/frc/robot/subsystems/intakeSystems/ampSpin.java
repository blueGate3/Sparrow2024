package frc.robot.subsystems.intakeSystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class ampSpin extends SubsystemBase {
    private Timer finalTimer = new Timer();
    private boolean isTrue = false;
    public void slowRotation(boolean isTrue){
        this.isTrue = isTrue;
    }
    @Override
    public void periodic(){
        if (isTrue){
            finalTimer.start();
                        if (finalTimer.get() < .5){
                            RobotContainer.m_IntakeRollerSubsystem.setMotor(.20); // correc direction
                        }else{
                            RobotContainer.m_IntakeRollerSubsystem.setMotor(0);
                            RobotContainer.m_IntakeRollerSubsystem.setMotor(-1);
                            finalTimer.stop();
                            finalTimer.reset();
                            isTrue = false;
                        }
            }
    }
}