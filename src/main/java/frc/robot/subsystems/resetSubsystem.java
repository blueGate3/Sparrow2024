package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.intake;

public class resetSubsystem extends SubsystemBase {
    public static boolean intakeHasNode = false;
    public static boolean shooterHasNode = false;
    public static boolean isHome = false;
    public static boolean beamBreakerDetection = false;
    // @Override
    // public void periodic(){
    //     SmartDashboard.putBoolean("intakeHasNode", intakeHasNode);
    //     SmartDashboard.putBoolean("shooterHasNode", shooterHasNode);
    //     SmartDashboard.putBoolean("isHome", isHome);
    //     SmartDashboard.putBoolean("beamBreakerDetection", beamBreakerDetection);
    // }
}