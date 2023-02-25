
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase{
    
    private CANSparkMax m_leftDrive;
    private CANSparkMax m_leftDriveFollower;
    private CANSparkMax m_rightDrive;
    private CANSparkMax m_rightDriveFollower;
    private DifferentialDrive m_robotDrive;

    public DriveBase()
    {
        m_leftDrive = new CANSparkMax(1, MotorType.kBrushed);
        m_leftDriveFollower = new CANSparkMax(2, MotorType.kBrushed);
        m_rightDriveFollower = new CANSparkMax(4, MotorType.kBrushed);
        m_rightDrive = new CANSparkMax(3, MotorType.kBrushed);
        m_rightDrive.setInverted(true);
        m_rightDriveFollower.setInverted(true);
        
        m_leftDriveFollower.follow(m_leftDrive);
        m_rightDriveFollower.follow(m_rightDrive);
        
        m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
        
        m_leftDrive.setIdleMode(IdleMode.kCoast);
        m_leftDriveFollower.setIdleMode(IdleMode.kCoast);
        m_rightDrive.setIdleMode(IdleMode.kCoast);
        m_rightDriveFollower.setIdleMode(IdleMode.kCoast);
        
        m_leftDrive.setSmartCurrentLimit(40, 15);
        m_rightDrive.setSmartCurrentLimit(40, 15);
        m_leftDriveFollower.setSmartCurrentLimit(40, 15);
        m_rightDriveFollower.setSmartCurrentLimit(40, 15);
    }

    public void arcadeDrive(double xSpeed, double zRotation) 
    {
        m_robotDrive.arcadeDrive(xSpeed, zRotation);
    }

}
