package frc.robot.Util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/*
	* A Class to "create a TalonSRX Encoder"
	*/
	public class SRXWrapper extends WPI_TalonSRX
    {
		public static final double conversionFactor = (0.4788 / 10.71);

		public SRXWrapper (int TalonID)
		{	
			super(TalonID);
		}

		//velocity
		public double getSelectedSensorVelocity(){
			return super.getSelectedSensorVelocity() * conversionFactor;
		}

		public double getSelectedSensorPosition(){
			return super.getSelectedSensorPosition() * conversionFactor;
		}

		public void set(double sensorVelocity){
			super.set(sensorVelocity*conversionFactor);
		}

		public ErrorCode setSelectedSensorPosition(double sensorPose){
			return super.setSelectedSensorPosition(sensorPose * conversionFactor);
		}

	}