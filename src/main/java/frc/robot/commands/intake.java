package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.resetSubsystem;

public class intake extends Command{
    private boolean state = false;
    public void initialize(){
        resetSubsystem.shooterHasNode = false;
        
        RobotContainer.m_ShooterRollersSubsystem.setShooterMotors(0);
        RobotContainer.m_ShooterRollersSubsystem.setIndexMotors(0, false);
        RobotContainer.m_IntakeRollerSubsystem.setMotor(0);
        RobotContainer.m_IntakeArmSubsystem.setIntakeArmPosition(-10.833388328552246);
        RobotContainer.m_IntakeRollerSubsystem.activateRollers(true);
        state = true;
    }
    public boolean isFinished(){
        return state;
    }
    public void end(){}
}