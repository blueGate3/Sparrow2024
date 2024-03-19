// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj2.command.button.Button;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.cameraserver.CameraServer;


import java.util.HashSet;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Timer;


//import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.net.PortForwarder;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.commands.CombinedCommands.CalibrateStuff;
//import frc.robot.commands.auto.AutoBuilder;
import frc.robot.variables.Motors;
import frc.robot.variables.Objects;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot{
    /**
     * Main robot functions
     */
    //private RobotContainer m_RobotContainer;
    private Command m_autonomousCommand;
    //private Autonomous autonomous = new Autonomous();
    private Command m_autoBuilder;
    private RobotContainer m_RobotContainer = new RobotContainer();
    
    private Timer m_autoDriveTimer = new Timer();
    
    Thread m_visionThread;
    //Drivetrain drivetrain = new Drivetrain();
   // DriveAndOperate driveAndOperate = new DriveAndOperate();
    private double autoV = 2.5; //auto constraints
    private double autoA = 1.25;
    private boolean start = false;
    
    

    

    /**
     * Robot variables
     */
    public String autoSelected;

    @Override
    public void robotInit() {
    
     //m_RobotContainer.setDefaultCommands();
     m_RobotContainer.configureButtonBindings();;
    
  


      



      

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic(){
        
    }

    @Override
    public void robotPeriodic() {
        m_RobotContainer.configureButtonBindings();
        
    // m_RobotContainer.setDefaultCommands();
        
        

    }

    @Override
    public void autonomousInit() {

        //m_RobotContainer.driveAutoPath(); 
        
    }

    @Override
    public void autonomousPeriodic() {
        
        
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        

        
     //m_RobotContainer.setDefaultCommands();

        
        m_RobotContainer.configureButtonBindings();

         
        //m_RobotContainer.m_ShooterArmSubsystem.resetEncoderPosition();
        
    }

    @Override
    public void teleopPeriodic() {
        
        // driveAndOperate.readDriverController();
        // driveAndOperate.readOperatorController();
        // // driveAndOperate.testJoystickRead();
        // driveAndOperate.driveAndOperate();
        //SmartDashboard.putNumber("xVisions", m_RobotContainer.visionSubsystem.getVisionTags(2)[1]);
        
     m_RobotContainer.configureButtonBindings();;
       
        
    }
    

}









//with love, blueGate3