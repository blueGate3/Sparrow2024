package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class spool extends Command {
    public spool(){}
    public void initialize(){
        RobotContainer.m_ShooterRollersSubsystem.setShooterMotors(0);
        RobotContainer.m_ShooterArmSubsystem.positionArmShooter(0.623319015752011, true);
    }
    public boolean isFinished(){
        return true;
    }
}