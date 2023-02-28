package Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class Auto extends CommandBase
{
    
    double odoX;
    double odoY;
    Rotation2d robotRotationDeg;
    DriveBase driveBase;

    public Auto(DriveBase driveBase)
    {
        System.out.println("StraightDrive Construction Started");
        driveBase = driveBase;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveBase);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() 
    {
        driveBase.resetBot_PosRot();
        System.out.println("StraightDrive Auto Initialized");
    }

    @Override
    public void execute() 
    {
        odoX = driveBase.getPose().getX();
        odoY = driveBase.getPose().getY();
        robotRotationDeg = driveBase.getPose().getRotation();
        double rotDouble = robotRotationDeg.getDegrees();

        driveBase.arcadeDrive(0.5, 0);


    }

    @Override
    public void end(boolean interrupted) 
    {
        driveBase.arcadeDrive(0, 0);
        System.out.println("Auto Finished");
        driveBase.resetBot_PosRot();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() 
    {
        return (odoX == 3);
        
        
    }


}
