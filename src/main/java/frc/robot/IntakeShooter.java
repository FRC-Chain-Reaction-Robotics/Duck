package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid; // imports methods for controlling double solenoid. Double rather than single beacuse air goes in two ways
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // shows various drive data
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // indicates this is a subsystem class (see: extends SubsystemBase)

public class IntakeShooter extends SubsystemBase
{
    // CANSparkMax can = new CANSparkMax(6, MotorType.kBrushless);  
    TalonSRX can = new TalonSRX(3); // instantiation

<<<<<<< HEAD
    DoubleSolenoid ds = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 4);
=======
    DoubleSolenoid ds = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
>>>>>>> 43bd07b930ebf48172535a2c911113ae195d19cc
    Compressor comp = new Compressor(PneumaticsModuleType.CTREPCM);
  
    public IntakeShooter() // constructor; like any class
    {
        ds.set(Value.kForward); //enables toggling
    }
    
    public void intakeInward()
    {
        can.set(ControlMode.PercentOutput, 1); // motor rotates cw/forward (pos 1)
    }

    public void intakeStop()
    {
        can.set(ControlMode.PercentOutput, 0); // Stops the intake
    }
    
    public void intakeOutwards()
	
    {
        can.set(ControlMode.PercentOutput, -1); // motor rotates ccw/backward (neg 1)
    }

    @Override
    public void periodic()
	{
        SmartDashboard.putBoolean("Pressure Switch", comp.getPressureSwitchValue());
        SmartDashboard.putBoolean("Intake is UP", ds.get() != DoubleSolenoid.Value.kForward);
    }

    public void togglePneumatics()
	{
        ds.toggle();
    }
}