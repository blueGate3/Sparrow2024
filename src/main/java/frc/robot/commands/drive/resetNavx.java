package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class resetNavx extends Command{
    public void initialize(){
        Drivetrain.navx.reset();
        SmartDashboard.putNumber("getAngle", Drivetrain.navx.getAngle());

    }
    public boolean isFinished(){
        return true;
    }
    
}
