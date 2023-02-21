package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid; // imports methods for controlling double solenoid. Double rather than single beacuse air goes in two ways
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // shows various drive data
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase; // indicates this is a subsystem class (see: extends SubsystemBase)
import edu.wpi.first.wpilibj.XboxController;
public class IntakeShooter extends SubsystemBase
{
    // CANSparkMax can = new CANSparkMax(6, MotorType.kBrushless);  
    TalonSRX can = new TalonSRX(3); // instantiation

    DoubleSolenoid ds = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 4);
    Compressor comp = new Compressor(PneumaticsModuleType.CTREPCM);
    XboxController driverController = new XboxController(0);

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
      /* var togglePnuematics = new JoystickButton(driverController, XboxController.Button.kA.value);
    var in = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    var out = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    togglePnuematics.whenPressed(new InstantCommand(intake::togglePneumatics));
    in.whileHeld(new RunCommand(intake::intakeInward, intake))
        .or(out.whileHeld(new RunCommand(intake::intakeOutwards, intake)))
        .whenInactive(new RunCommand(intake::intakeStop, intake));
    //B button to test motors
    var motorTestButton = new JoystickButton(driverController, XboxController.Button.kB.value);
    //motorTestButton.whenPressed(new InstantCommand(()-> (new WPI_TalonSRX(5)).set(3), intake));  */
 /*  if (driverController.getLeftBumperPressed()) {
  ds.setValue(DoubleSolenoid.value(kForward));
} else if(driverController.	getRightBumperPressed()) { 
  ds.setValue(DoubleSolenoid.value(kReverse));
} */

  if (driverController.getLeftBumperPressed()) {
          ds.set(Value.kForward); //enables toggling
} else if(driverController.	getRightBumperPressed()) { 
          ds.set(Value.kReverse); //enables toggling

}
ds.toggle(); 
    


    }
}