package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends CommandBase{
    private final DriveBase m_subsystem;
    Joystick driverJoystick;

    public TankDrive(DriveBase subsystem) 
    {
        m_subsystem = subsystem;
    
        driverJoystick = new Joystick(0);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    
    
    @Override
    public void execute() 
    {
        m_subsystem.arcadeDrive(driverJoystick.getRawAxis(1), driverJoystick.getRawAxis(0));
    }

}
