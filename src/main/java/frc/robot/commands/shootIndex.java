package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class shootIndex extends Command {
    public void initialize(){
        RobotContainer.m_ShooterRollersSubsystem.setIndexMotors(1, false);
        // RobotContainer.m_IntakeRollerSubsystem.setMotor(1);
    }
    public boolean isFinished(){
        return true;
    }
    
}