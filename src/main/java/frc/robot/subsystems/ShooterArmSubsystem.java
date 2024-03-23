// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;


// public class ShooterArmSubsystem extends SubsystemBase {
//     CANSparkMax armMotor = new CANSparkMax(12, MotorType.kBrushless);
//     PIDController m_PIDController = new PIDController(0.001, 0.005, 0.0000075); 
//     RelativeEncoder armMotorEncoder = armMotor.getEncoder();
//     private int gearRatio = 65;
//     private double rollerSpeed = 0.05;

//     public void setArmPosition(double pos){ 
//         double currentRelativePosition = armMotorEncoder.getPosition()*gearRatio; // can change to sparkpid
//         double voltage = m_PIDController.calculate(currentRelativePosition, pos);
//         armMotor.set(voltage);
//         m_PIDController.setTolerance(.5);
//     }
    
//     public void resetEncoderPosition(){
//         armMotorEncoder.setPosition(0);
//     }

//     public void setBreakModeStatus(boolean status){
//         if (status){
//             armMotor.setIdleMode(IdleMode.kBrake);
//         }else{
//             armMotor.setIdleMode(IdleMode.kCoast);
//         }

//     }
//     @Override
//     public void periodic() {
//       SmartDashboard.putNumber("relativeEncoderPosition", armMotorEncoder.getPosition()*gearRatio);
//     }
  
// }
