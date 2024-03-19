// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.FollowPathCommand;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */

public class Drivetrain extends SubsystemBase {
    
    //private VisionSubsystem m_visionSubsystem;// = new VisionSubsystem();

    




    
    // kMaxSpeed was 2 AND kmaxangularspeed was pi/3 (before testing [district champs])
    // SOLID SPEEDS 3.25 M/S /AND PI/2.25 ROT/S
    public static final double kMaxSpeed = 3.25; // 3.68 meters per second or 12.1 ft/s (max speed of SDS Mk3 with Neo motor)
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    public static final AHRS navx = new AHRS();
    public boolean donePlace = false;
    //private PIDController xController = new PIDController(1/100, 0, 0);
    //private PIDController yController = new PIDController(1/100, 0, 0);
    private double timerNow = 0;
    private double timerPrevious = 0;
    private double anglePreviousRoll = 0;
    private double anglePreviousPitch = 0;
    private double xControl = 0;
    private double yControl = 0;
    public double kPBal = .01485; //.014
    public double kDBal = 0.001;//.0019;


    //positions of each swerve unit on the robot 
    // private final Translation2d m_frontLeftLocation = new Translation2d(-0.538,  0.538);
    // private final Translation2d m_frontRightLocation = new Translation2d(0.538,  0.538);
    // private final Translation2d m_backLeftLocation = new Translation2d( -0.538, -0.538);
    // private final Translation2d m_backRightLocation = new Translation2d( 0.538, -0.538);

    // //constructor for each swerve module
    // private final SwerveModule m_frontRight = new SwerveModule(21, 2, 10, 0.564);
    // private final SwerveModule m_frontLeft  = new SwerveModule(3, 4, 11, 0.563);
    // private final SwerveModule m_backLeft   = new SwerveModule(5, 6, 12, 0.875); //0.05178
    // private final SwerveModule m_backRight  = new SwerveModule(7, 8, 13, 0.921);
    //i am so dumb im a failure 
    // Locations pf each swerve module relative to the center of the robot
    private final Translation2d m_frontRightLocation = new Translation2d( 0.2667, 0.2667); // set for 10.5 inches, that's respectable ig
    private final Translation2d m_frontLeftLocation = new Translation2d(0.2667,  -0.2667);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.2667,  -0.2667);
    private final Translation2d m_backRightLocation = new Translation2d( -0.2667, 0.2667); //Was using hypotenuse before, but I flipped negatives to test. This is how I think it's SUPPOSED to be done, not sure though. Try making this double negative and the one above it a y-positive only if it isnt aligned. 
    //also try doubling it if it's still off, thats what last year has it as so :| (thats a face for when i confuse myself).

    // Constructor for each swerve module (0.49461688736542214)
    private final SwerveModule m_frontRight = new SwerveModule(2, 1, 12, 0.9091540727288518, false, false); //
    private final SwerveModule m_frontLeft = new SwerveModule(4, 3, 13, 0.11184375279609382, true, false); //.8100
    private final SwerveModule m_backLeft = new SwerveModule(6, 5, 18, 0.20064420501610514, true, false); //0.020214250505356263
    private final SwerveModule m_backRight = new SwerveModule(8, 7, 20, 0.1882500047062501, true, false); //0.82150

    // Swerve Drive Kinematics (note the ordering [frontRight, frontLeft, backLeft, backRight] [counterclockwise from the frontRight])
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontRightLocation, m_frontLeftLocation, m_backLeftLocation, m_backRightLocation);


    //INITIAL POSITIONS to help define swerve drive odometry. THis was a headache
    public SwerveModulePosition positionFrontLeft = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition positionFrontRight = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition positionBackLeft = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition positionBackRight = new SwerveModulePosition(0.0,new Rotation2d(0.0));
    public SwerveModulePosition[] initialPositions = {positionFrontRight, positionFrontLeft, positionBackLeft, positionBackRight};
    public SwerveDriveKinematics m_initialStates; 
    public SwerveModulePosition[] positions = new SwerveModulePosition[4];
    
    public final SwerveDriveOdometry m_odometry;
    
    public final SwerveModule Swerves[] = {m_frontRight, m_frontLeft, m_backLeft, m_backRight};

    


    
    // Constructor
    public Drivetrain() {
        // System.out.println(m_backLeft.m_TurnPWMEncoder.getOutput());
        m_initialStates = new SwerveDriveKinematics(m_frontRightLocation, m_frontLeftLocation, m_backLeftLocation, m_backRightLocation);

        m_odometry = new SwerveDriveOdometry(
            m_kinematics, 
            navx.getRotation2d(), initialPositions
        );
        // m_visionSubsystem = new VisionSubsystem();
        // Configure AutoBuilder last i added all this in.

        AutoBuilder.configureHolonomic(
            this::getCurrentPose2d, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveAutonomous, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }


    @Override
    public void periodic () {
        updateOdometry();
        //SmartDashboard.putNumber("xOdometry", getCurrentPose2d().getX());

    }

    /**
     * Place a cone using vision pose
     * Overrides the current odometry with the vision odometry
     */
    


    public void coneAutoPlace() {
        
    }

    

    /**
     * Manages the positioning process for auto placing a cone
     * Uses vision odometry to control the swerve motion
     */
    public void controlAutoPlace() {
        double maxSpeed = .2;
        double yPosition = .5; //in meters
        double xPosition = .2; ///in meters

        double yControl;
        double rotControl = -navx.getRotation2d().getRadians()/4;

        if (Math.abs(xControl) > maxSpeed) {
            if (xControl > 0) {
                xControl = maxSpeed;
            } else {
                xControl = -maxSpeed;
            }
        }
        
        

        
        // drive(kMaxSpeed, kMaxAngularSpeed, anglePreviousRoll, isBalanced(), isBalanced());
    }
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
//    public SwerveModulePosition[] initialPositions = {positionFrontRight, positionFrontLeft, positionBackLeft, positionBackRight};

     //possible paramaters:driverXStick, -driverYStick, -driverRotateStick,
     //used to be: (double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean defenseHoldingMode)
    @SuppressWarnings("ParameterName")
    public void drive(double driverXStick, double driverYStick, double driverRotateStick, boolean fieldRelative, boolean defenseHoldingMode) {
        // System.out.println("Front Right Enc: " + m_frontRight.m_TurnPWMEncoder.getOutput());
        // System.out.println("Front Left Enc: " + m_frontLeft.m_TurnPWMEncoder.getOutput());
        // System.out.println("Back Right Enc: " + m_backRight.m_TurnPWMEncoder.getOutput());
        // System.out.println("Back Left Enc: " + m_backLeft.m_TurnPWMEncoder.getOutput());

        //SmartDashboard.putNumber("X Speed", driverXStick);
        double offset = (navx.getAngle());//or get angle adjustment
        double realOffset = Math.toRadians(offset);//if the line below no worky add this in front //also it used to be navx.getRotation2d().getRadians()-offset or offset was realOffset
        Rotation2d robotRotation = new Rotation2d(realOffset); //+ angleOffset); //DriverStation.getAlliance() == Alliance.Blue ? new Rotation2d(navx.getRotation2d().getDegrees() + 180) : navx.getRotation2d();
        // SmartDashboard.putNumber ( "inputRotiation", robotRotation.getDegrees());
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(driverXStick, driverYStick, driverRotateStick, robotRotation): new ChassisSpeeds(driverXStick, driverYStick, driverYStick));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        if (!defenseHoldingMode) {
            m_frontRight.setDesiredState(swerveModuleStates[0]);
            m_frontLeft.setDesiredState(swerveModuleStates[1]);
            m_backLeft.setDesiredState(swerveModuleStates[2]);//NOTE FIX THESE TWO BACK ONES LATER WE MAY NEED THESE IN RIGHT ORDER IN AUTO.
            m_backRight.setDesiredState(swerveModuleStates[3]);
        } 
        else {
             m_backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(3*(Math.PI / 4))));
             m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d( (Math.PI / 4))));
             m_backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
             m_frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(3*(Math.PI / 4))));
         
    }

    }

    /**
     * Balance the robot on the charging station
     * Controls the drive 
     */
    // public void balanceRobot() { //Y IS NEGATIVE FOR SOME REASON
    //     // SmartDashboard.putNumber("Pitch", Objects.navx.getPitch());
    //     PIDCalculator(.02);
    //     drive(xControl, yControl, 0, false, false);
        
    // }

    /**
     * Update the odometry and vision poses
     */
    public void setPose (Pose2d pose) {
        m_odometry.resetPosition(new Rotation2d(pose.getRotation().getRadians()), initialPositions, pose);
        }
    
    /**
     * Drives with swerve during the autonomous period
     * @param chassisSpeeds
     */
    public void driveAutonomous (ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        m_frontRight.setDesiredState(moduleStates[0]);
        m_frontLeft.setDesiredState(moduleStates[1]);
        m_backLeft.setDesiredState(moduleStates[2]);
        m_backRight.setDesiredState(moduleStates[3]);
        
    }
    /**
     * Updates the position of the robot relative to where its starting position
     */
    public void updateOdometry() { //it may have to be in the right order
        positions[0] = new SwerveModulePosition(m_frontLeft.getDifferentState().speedMetersPerSecond, m_frontLeft.getState().angle);
        positions[1] = new SwerveModulePosition(m_backLeft.getDifferentState().speedMetersPerSecond, m_backLeft.getState().angle);
        positions[2] = new SwerveModulePosition(m_backRight.getDifferentState().speedMetersPerSecond, m_backRight.getState().angle);
        positions[3] = new SwerveModulePosition(m_frontRight.getDifferentState().speedMetersPerSecond, m_frontRight.getState().angle);
       

        Pose2d m_distance = m_odometry.update(navx.getRotation2d(), positions);        

        // for (int i = 0; i < Swerves.length; i++ ){
        //     Swerves[i].getEncoderData();
        // }
        //System.out.println("");

        // SmartDashboard.putNumber("VisRotation", getCurrentPose2d().getRotation().getDegrees());
        
        
        // SmartDashboard.putNumber("Odometry X", m_distance.getX());
        // SmartDashboard.putNumber("Odometry Y", m_distance.getY());
        // SmartDashboard.putNumber("Rotation", getCurrentPose2d().getRotation().getDegrees());
        // // m_odometry.update(Objects.navx.getRotation2d(), new SwerveModulePosition[] {
            // m_frontRight.getState()
        // });
        // m_odometry.update(Objects.navx.getRotation2d(), m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState());
     
    }

    /**
     * Reset the navx and set a starting angle
     * @param initialAngle starting angle
     */
    public Command resetNavxMark (double initialAngle) {
        navx.reset(); //90 because of the feild orientation vs our driver fov
        navx.setAngleAdjustment(-initialAngle); //TODO negative because navx has a goofy coordinate system
        return null;
    }

    public void zeroHeading() {
        navx.reset();
    }
    /**
     * Gives the current position and rotation of the robot (meters) based on the wheel odometry from where the robot started
     * @return Pose2d of current robot position
     */
    public Pose2d getCurrentPose2d() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Gives the current pose of the robot based on the limelight vision pose
     */
    

    /**
     * Converts raw module states into chassis speeds
     * @return chassis speeds object
     */
    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(m_backLeft.getState(), m_frontLeft.getState(), m_backRight.getState(), m_frontRight.getState());
    }



    /**
     * Custom PID controller that observes the rate at which the pitch and roll are changing
     * @param dt double - change in time
     */
    public void PIDCalculator (double dt) {
        double thisPitch = -navx.getPitch();
        double thisRoll = navx.getRoll();
        double dPitch = thisPitch - anglePreviousPitch; 
        double dRoll = thisRoll - anglePreviousRoll;
    
        xControl = -thisPitch*kPBal; //+ (dPitch/dt) * kDBal; //xController.calculate(-Objects.navx.getPitch(), 0);//(-Objects.navx.getPitch()*kPBal); 
        yControl = -thisRoll*kPBal;// + (dRoll/dt) * kDBal;//yController.calculate(Objects.navx.getRoll(), 0);//(*kPBal);
        anglePreviousPitch = thisPitch;
        anglePreviousRoll = thisRoll;
        // SmartDashboard.putNumber("PitchGyro", dPitch/dt);
        // SmartDashboard.putNumber("Rollgyro", dRoll/dt);
        
    }

    /**
     * Returns whether the robot is balanced on the charging station
     * If the rate of charge of the gyro in the x or y direction is at least 10, the robot is assumed to be balanced
     * @return is the robot balanced
     */
    public boolean isBalanced() {
        // return /**(Math.abs(navx.getPitch())<= 5 && Math.abs(navx.getRoll())<=5 ) && */ (Math.abs(navx.getRawGyroX()) >= 10 || (Math.abs(navx.getRawGyroY()) >= 10));
        return Math.abs(navx.getRawGyroX()) >= 10 || (Math.abs(navx.getRawGyroY()) >= 10);
      }

      public boolean getAlliance() {
        
              var alliance = DriverStation.getAlliance();
              
                return alliance.get() == DriverStation.Alliance.Red;
              
      }

      public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathHolonomic(
                path,
                this::getCurrentPose2d, // Robot pose supplier
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveAutonomous, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                     return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }




    


}