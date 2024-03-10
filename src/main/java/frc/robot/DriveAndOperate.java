package frc.robot;

import frc.robot.variables.Motors;
import frc.robot.variables.Objects;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.RestoreAction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.commands.drive.BalanceCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.DoubleSolenoid;


public class DriveAndOperate {

    // private final XboxController m_controller = new XboxController(0);
    private final Joystick m_DriverController = new Joystick(0);
    public final Joystick m_OperatorController = new Joystick(1);

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(10);


    /**
     * State of all buttons
     */
    private double climbRotateSpeed = 0;
    private boolean rotateDown = false;
    private double driverXStick = 0;
    private double driverYStick = 0;
    private double driverRotateStick = 0;
    private boolean intakeButton = false;
    private boolean shootWithVisionButton = false;
    private boolean pixyLineUp = false; 
    private boolean spoolUpButton = false;

    //private double driverLeftHood = 0;
    private double testHood = 0;
    private boolean visionShoot = false;
    private double driverShootThrottle = 0;
    private boolean ejectBall = false;
    private boolean rotateBackwardButton = false;
    private boolean rotateForwardButton = false;
    private boolean lowGoal = false;
    private boolean resetNavx = false;
    private boolean manualButton = false;
    private boolean raiseClimb = false;
    private boolean robotRelative;
    private boolean defenseHoldingMode;
    private double idealPixyY = 155; //was 174
    private double idealPixyX = 167;
    private double xPositionError = 0;
    private double yPositionError = 0;
    private double climbRaiseSpeed = 0;
    
    private Trigger balanceButton = new JoystickButton(m_DriverController, 1);

    /**
     * Runs subsystems on the robot based on pre-evaluated driver and operator inputs
     */
    public void driveAndOperate () {
        double xSpeed = m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverXStick, 0.05)) * Drivetrain.kMaxSpeed;
        double ySpeed = m_yspeedLimiter.calculate(MathUtil.applyDeadband(-driverYStick, 0.05)) * Drivetrain.kMaxSpeed;
        double rot = m_rotLimiter.calculate(-MathUtil.applyDeadband(driverRotateStick, 0.05)) * Drivetrain.kMaxAngularSpeed;

        boolean fieldRelative = true;
        
        /**
         * RESET NAVX
         */
        if (resetNavx) {
            
        }

        if (Math.abs(xSpeed)<.015) {
            xSpeed = 0;
        }
        if (Math.abs(ySpeed)<.015) {
            ySpeed = 0;
        }      
        if (Math.abs(rot)<.015) {
            rot = 0;
        }
        /**
         * DRIVE THE ROBOT
         */
        if (!balanceButton.getAsBoolean()) {
            //Objects.driveSubsystem.balanceRobot();
            //Objects.driveSubsystem.driveSwerve(xSpeed*.3, ySpeed*.3, (rot +.0001 )/2, fieldRelative, defenseHoldingMode); //final movement; sends drive values to swerve
        }
        //RobotContainer.drivetrain.updateOdometry(); //where are we? --- idk, we're all lost
    }

    
    /**
     * Reads the driver controller buttons and stores their current state
     */
    public void readDriverController() {
        
        // Trigger balanceButton = new Trigger(m_DriverController.getRawButton(1));
        }
    
    /**
     * Reads the operator controller buttons and stores their current state
     */
    public void readOperatorController() {
        // spoolUpButton = m_OperatorController.getRawButton(1); // a button
        // ejectBall = m_OperatorController.getRawButton(2); // b button
       
        //  //center buttons menu and hamburger buttons
        // manualButton = m_OperatorController.getRawButton(3); //manual index x button
        // intakeButton = m_OperatorController.getRawButton(6); //right bumper
        // if (!raiseClimb) {
        //     raiseClimb = m_OperatorController.getRawButton(8) && m_OperatorController.getRawButton(7);
        // }

        // climbRaiseSpeed = -m_OperatorController.getRawAxis(1);

        // climbRotateSpeed = (m_OperatorController.getRawAxis(3) - m_OperatorController.getRawAxis(2));
        
        
        
        // if (shootWithVisionButton) {
        //     Objects.hoodSubsystem.adjustHood(testHood);
        //     Objects.shootSubsystem.setShooterRPM(driverShootThrottle);
        // } else {
        //     Motors.shooterLeader.stopMotor();
        //     Motors.shooterFollower.stopMotor();
        // }
        /**
         * OTHER CLIMB
         */


        //old climb
        // if (m_OperatorController.getPOV() == 0) {
        //     releaseTopHooks = true;
        //     releaseBottomHooks = false;
        // }
        // else if (m_OperatorController.getPOV() == 180) {
        //     releaseBottomHooks = true;
        //     releaseTopHooks = false;
        // }
        // else {
        //     releaseTopHooks = false;
        //     releaseBottomHooks = false;
        // }


        // //old climb
        // if (m_OperatorController.getPOV() == 90) {
        //     rotateForwardButton = true;
        //     rotateBackwardButton = false;
        // }
        // else if (m_OperatorController.getPOV() == 270) {
        //     rotateBackwardButton = true;
        //     rotateForwardButton = false;
        // }
        // else {
        //     rotateForwardButton = false;
        //     rotateBackwardButton = false;
        // }
        // climbRotateSpeed = m_OperatorController.getRawAxis(3) - m_OperatorController.getRawAxis(2);
        
    }

    public void testJoystickRead () {
        // driverShootThrottle = (((m_DriverController.getRawAxis(3)+1))*1000 + 1000) * (25/9);
        // testHood = (m_OperatorController.getRawAxis(3)+1)*.1;
        // resetHood = m_DriverController.getRawButton(5);
        // shootWithVisionButton = m_DriverController.getRawButton(1);
        // climbRotateSpeed = m_OperatorController.getRawAxis(3) - m_OperatorController.getRawAxis(2);
        // intakeButton = m_OperatorController.getRawButton(6); //right bumper

    }

}