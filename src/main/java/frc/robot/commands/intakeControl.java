package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.resetSubsystem;

public class intakeControl extends Command {
    private boolean state = false;
    public void initialize(){
        if (!resetSubsystem.isHome){
            RobotContainer.m_IntakeRollerSubsystem.setMotor(-1);
            state = true;
        }
    }
    public boolean isFinished(){
        return state;
    }
    public void end(){}
}