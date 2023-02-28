package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.SRXWrapper;

public class Drive extends SubsystemBase{
    WPI_TalonSRX leftFront = new SRXWrapper(Constants.Drivetrain.kLFMotorID);
	WPI_TalonSRX leftBack = new SRXWrapper(Constants.Drivetrain.kLBMotorID);
	WPI_TalonSRX rightFront = new SRXWrapper(Constants.Drivetrain.kRFMotorID);
	WPI_TalonSRX rightBack = new SRXWrapper(Constants.Drivetrain.kRBMotorID);

	//RelativeEncoder lfEncoder = leftFront.getEncoder();
	//RelativeEncoder lbEncoder = leftBack.getEncoder();
	
	//RelativeEncoder rfEncoder = rightFront.getEncoder();
	//RelativeEncoder rbEncoder = rightBack.getEncoder();

	MotorControllerGroup left = new MotorControllerGroup(leftFront, leftBack);
	MotorControllerGroup right = new MotorControllerGroup(rightFront, rightBack);;

	DifferentialDrive dt = new DifferentialDrive(left, right);

	Field2d field = new Field2d();

	// Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	AHRS gyro = new AHRS(SPI.Port.kMXP);

	private static final double output = 1.0;

	
	
	TalonSRXConfiguration config = new TalonSRXConfiguration();

	private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(	new Rotation2d(), 0, 0, new Pose2d());

	public Drive()
	{

		//Taken care of in the SRXWrapper Class
		// lfEncoder.setPositionConversionFactor(0.4788 / 10.71);
		// rfEncoder.setPositionConversionFactor(0.4788 / 10.71);
		// lfEncoder.setVelocityConversionFactor(0.4788 / 10.71);
		// rfEncoder.setVelocityConversionFactor(0.4788 / 10.71);

		leftFront.setSelectedSensorPosition(0);
		leftBack.setSelectedSensorPosition(0);
		rightFront.setSelectedSensorPosition(0);
		rightBack.setSelectedSensorPosition(0);
		

		leftFront.setNeutralMode(NeutralMode.Coast);
		rightFront.setNeutralMode(NeutralMode.Coast);
		leftBack.setNeutralMode(NeutralMode.Coast);
		rightBack.setNeutralMode(NeutralMode.Coast);
			
		config.continuousCurrentLimit = Constants.Drivetrain.kGoodFreeCurrentLimit;
        config.peakCurrentLimit = Constants.Drivetrain.kGoodStallCurrentLimit;

        configAllSettings();

		leftFront.setInverted(false);
		leftBack.setInverted(false);
		rightFront.setInverted(true);
		rightBack.setInverted(true);

		//Burn Flash Method is replaced by configAllSettings

		SmartDashboard.putData(field);
		setOutput(1);
		
		// evilMode();
	}

	public void arcadeDrive(double xSpeed, double zRotation)
	{
		dt.arcadeDrive(output * xSpeed, output * zRotation);
	}

	public double getAngle()
	{
		return gyro.getAngle();
	}

	public void resetSensors()
	{
		resetEncoders();
		resetGyro();
	}


	public void setOutput(double x)
	{
		dt.setMaxOutput(x);
	}

	public void resetEncoders()
	{
		leftFront.setInverted(false);
		leftBack.setInverted(false);
		rightFront.setInverted(true);
		rightBack.setInverted(true);

	}

	public void resetGyro()
	{
		gyro.reset();
		gyro.calibrate();
	}

	@Override
	public void periodic()
	{
		// This method will be called once per scheduler run'

		/*
		SmartDashboard.putNumber("lTicks", lfEncoder.getSelectedSensorPosition()());
		SmartDashboard.putNumber("rTicks", rfEncoder.getSelectedSensorPosition()());
		// Print out the odometry to smartdashboard
		var odometryPose = odometry.getPoseMeters();
		SmartDashboard.putNumber("odom x", odometryPose.getTranslation().getX());
		SmartDashboard.putNumber("odom y", odometryPose.getTranslation().getY());
		SmartDashboard.putNumber("odom heading", odometryPose.getRotation().getDegrees());
		SmartDashboard.putNumber("gyro raw angle", gyro.getAngle());
		SmartDashboard.putNumber("dtd dist", getDistanceMeters());
		SmartDashboard.putNumber("left speeds", getSelectedSensorVelocity().leftMetersPerSecond);
		SmartDashboard.putNumber("right speeds", getSelectedSensorVelocity().rightMetersPerSecond);
		field.setRobotPose(getPose());
		odometry.update(gyro.getRotation2d(), lfEncoder.getSelectedSensorPosition()(), rfEncoder.getSelectedSensorPosition()());
		*/
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds()
	{
		return new DifferentialDriveWheelSpeeds();
		//return new DifferentialDriveWheelSpeeds(lfEncoder.getVelocity(), rfEncoder.getVelocity());
	}

	public Pose2d getPose()
	{
		return odometry.getPoseMeters();
	}

	/**
	 * Controls the left and right sides of the drive directly with voltages.
	 *
	 * @param leftVolts
	 *            the commanded left output
	 * @param rightVolts
	 *            the commanded right output
	 */
	public void tankDriveVolts(double leftVolts, double rightVolts)
	{
		left.setVoltage(-leftVolts);
		right.setVoltage(rightVolts);
		dt.feed();
	}

	public void resetOdometry(Pose2d initialPose)
	{
		odometry.resetPosition(new Rotation2d(gyro.getAngle()), 0, 0, initialPose);
	}

	public double getDistanceMeters()
	{
		// return 0.0;
		return Math.max(leftFront.getSelectedSensorPosition(), rightFront.getSelectedSensorPosition());
	}

	public void evilMode()
	{
		leftFront.setNeutralMode(NeutralMode.Brake);
		rightFront.setNeutralMode(NeutralMode.Brake);
		leftBack.setNeutralMode(NeutralMode.Brake);
		rightBack.setNeutralMode(NeutralMode.Brake);

		config.continuousCurrentLimit = Constants.Drivetrain.kEvilFreeCurrentLimit;
        config.peakCurrentLimit = Constants.Drivetrain.kEvilStallCurrentLimit;

        configAllSettings();
	}

	public void goodMode()
	{
		leftFront.setNeutralMode(NeutralMode.Coast);
		rightFront.setNeutralMode(NeutralMode.Coast);
		leftBack.setNeutralMode(NeutralMode.Coast);
		rightBack.setNeutralMode(NeutralMode.Coast);
		
		config.continuousCurrentLimit = Constants.Drivetrain.kGoodFreeCurrentLimit;
        config.peakCurrentLimit = Constants.Drivetrain.kGoodStallCurrentLimit;

        configAllSettings();
	
	}

	public void slowMode()
	{
		config.continuousCurrentLimit = Constants.Drivetrain.kEvilFreeCurrentLimit;
        config.peakCurrentLimit = Constants.Drivetrain.kEvilStallCurrentLimit;

        configAllSettings();
        
		setOutput(0.3);
	}

	SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.Drivetrain.ksVolts, Constants.Drivetrain.kvVoltSecondsPerMeter, Constants.Drivetrain.kaVoltSecondsSquaredPerMeter);
  	PIDController lPid = new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0);
  	PIDController rPid = new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0);
  	
	public void setWheelSpeeds(double left, double right)
  	{
		DifferentialDriveWheelSpeeds wheelSpeedNow = getWheelSpeeds();
    	double leftVolt  =	lPid.calculate(wheelSpeedNow.leftMetersPerSecond, left) +
							feedForward.calculate(left);
    	double rightVolt =  rPid.calculate(wheelSpeedNow.leftMetersPerSecond, right) +
							feedForward.calculate(right);
		tankDriveVolts(-leftVolt, rightVolt);
  	}

    private void configAllSettings()
    {
        WPI_TalonSRX[] motors = new WPI_TalonSRX[]
        {
            leftFront,
            leftBack,
            rightFront,
            rightBack,
        };

        for (WPI_TalonSRX motor: motors)
            motor.configAllSettings(config);
    }
}