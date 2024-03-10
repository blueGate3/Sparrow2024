package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AutoPlace extends Command {
    private Drivetrain m_drivetrain;
    public AutoPlace (Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    } 
    
    @Override 
    public void initialize () {
        // m_drivetrain.instantiatePlaceOdometry();
    }

    @Override 
    public void execute() {
        m_drivetrain.controlAutoPlace();
    }

    @Override 
    public boolean isFinished() {
        //return m_drivetrain.donePlace;
        return false;

    }
}