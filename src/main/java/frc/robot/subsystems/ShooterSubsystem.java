// package frc.robot.subsystems;

// import org.ejml.UtilEjml;

// import com.kauailabs.navx.frc.AHRS;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DigitalSource;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DutyCycle;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;


// public class ShooterSubsystem extends SubsystemBase {
//     // Declaration of variables
//     private int lowerMotorPort = 25 ; //13 is for intake testing should be negative generally 14
//     private int upperMotorPort = 16; //14 is the bottom generally 13
//     private int feedMotor = 10;
//     private Timer timer = new Timer();

//     private CANSparkMax feedShootMotor = new CANSparkMax(feedMotor, MotorType.kBrushless); 
//     private CANSparkMax upperShootMotor = new CANSparkMax(upperMotorPort, MotorType.kBrushless);
//     private CANSparkMax lowerShootMotor = new CANSparkMax(lowerMotorPort, MotorType.kBrushless); //9 is lower 11 is upper 10 is the one that pushes it into that
    
    
//     //unsure how necesary it is to have three
//     private SparkPIDController m_pidController = lowerShootMotor.getPIDController();
    
//     public ShooterSubsystem() {

//     }
    
//     @Override
//     public void periodic() {


// }

//     //called on in ConfigureButtonBindings and our auto, this is just the feed motor because the motors are already spooled up. 
//     public void Shoot (int shootCommandType) {
        
//         //runs the spool up motors
//         if(shootCommandType == 0){
            
//             lowerShootMotor.set(1.0);
//             upperShootMotor.set(-1);
//         }

//         //no shooting, deactivates all motors
//         if (shootCommandType == 1){
//             lowerShootMotor.set(0);
//             upperShootMotor.set(0);
//             feedShootMotor.set(0);
//         }

//         if (shootCommandType == 2) {
//             feedShootMotor.set(1); 
//         }

//         if (shootCommandType ==3) {
//             timer.start();
//             double time = Timer.getFPGATimestamp();
//             double timeToRun = 2;
//             while (Timer.getFPGATimestamp() <= timeToRun){
//             feedShootMotor.set(1);
                
//             if (time > timeToRun) {
//                 timer.reset();
//             }
//             }

//         }
              
//     }
    
// }
