// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.GenericHID;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.*;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
public class Robot extends TimedRobot {
  // private final MotorController m_leftMotor = new MotorControllerGroup(new WPI_TalonSRX(1), new WPI_TalonSRX(3));
  // private final MotorController m_rightMotor = new MotorControllerGroup(new WPI_TalonSRX(2), new WPI_TalonSRX(4));
  // private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  // private final CommandXboxController m_driverController = new CommandXboxController(0);

  private RobotContainer m_robotContainer;
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightMotor.setInverted(true);
    // m_robotDrive.setMaxOutput(1.0);
    // m_driverController.rightBumper().whileTrue(new RunCommand(() -> m_robotDrive.arcadeDrive(-m_driverController.getLeftY() * 0.25
    // , -m_driverController.getRightX() * 0.25)));

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void teleopPeriodic() {
    // Drive with tank drive.
    // That means that the Y axis of the left stick moves the left side
    // of the robot forward and backward, and the Y axis of the right stick 
    //m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX());
    // m_driverController.rightBumper().whileTrue(new RunCommand(() -> m_robotDrive.setMaxOutput(.3)));
  }
}