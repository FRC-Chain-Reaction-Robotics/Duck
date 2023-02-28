// package frc.robot.Subsystems;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Drive extends SubsystemBase {
//     private WPI_TalonSRX leftFront = new WPI_TalonSRX(0);
//     private WPI_TalonSRX leftBack = new WPI_TalonSRX(1);
//     private WPI_TalonSRX rightFront = new WPI_TalonSRX(2);
//     private WPI_TalonSRX rightBack = new WPI_TalonSRX(3);
    
//     MotorControllerGroup left = new MotorControllerGroup(leftBack, leftFront);
//     MotorControllerGroup right = new MotorControllerGroup(rightBack, rightFront);
    
//     DifferentialDrive dt = new DifferentialDrive(left, right);
    
//     DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(.69);
//     DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0, new Pose2d());



//     public Drive() {
//         leftFront.setInverted(false);
//         leftBack.setInverted(false);    

        




//     }


// }
