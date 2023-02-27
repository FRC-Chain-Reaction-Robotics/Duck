// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.commands.TeleopSwerve;
import frc.robot.IntakeShooter;
//import frc.robot.subsystems.Swerve;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive dt;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private final MotorController m_leftMotor = new MotorControllerGroup(new WPI_TalonSRX(0), new WPI_TalonSRX(1));
  
  private final MotorController m_rightMotor = new MotorControllerGroup(new WPI_TalonSRX(2), new WPI_TalonSRX(3));


  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    dt = new DifferentialDrive(m_leftMotor, m_rightMotor);
    XboxController driverController = new XboxController(0);
<<<<<<< HEAD
=======

>>>>>>> e8e5889689b39ebdbb3c945e2b16e3defbe7af33
    IntakeShooter intake = new IntakeShooter();

  
    //dt.setDefaultCommand(new RunCommand(() -> dt.arcadeDrive(-driverController.getLeftY(),
        //driverController.getRightX()), dt));
<<<<<<< HEAD
=======


    // dt.setDefaultCommand( RunCommand(() -> dt.arcadeDrive(-driverController.getLeftY(),
        // driverController.getRightX()), dt));
>>>>>>> e8e5889689b39ebdbb3c945e2b16e3defbe7af33


   /* var togglePnuematics = new JoystickButton(driverController, XboxController.Button.kA.value);
    var in = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    var out = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
<<<<<<< HEAD

    CANSparkMax spark = new CANSparkMax(5, MotorType.kBrushless);
    JoystickButton j = new JoystickButton(driverController, XboxController.Button.kX.value);
    j.whenPressed(new InstantCommand(() -> spark.set(1)));
=======
    togglePnuematics.whenPressed(new InstantCommand(intake::togglePneumatics));
    in.whileHeld(new RunCommand(intake::intakeInward, intake))
        .or(out.whileHeld(new RunCommand(intake::intakeOutwards, intake)))
        .whenInactive(new RunCommand(intake::intakeStop, intake));
    //B button to test motors
    var motorTestButton = new JoystickButton(driverController, XboxController.Button.kB.value);
    //motorTestButton.whenPressed(new InstantCommand(()-> (new WPI_TalonSRX(5)).set(3), intake)); */
  

>>>>>>> e8e5889689b39ebdbb3c945e2b16e3defbe7af33
  }

  @Override
  public void teleopPeriodic() {
    dt.feed(); 
    IntakeShooter intake = new IntakeShooter(); 
    intake.togglePneumatics(); 

    
  
  }

}
