
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase{
    
    private CANSparkMax m_leftDrive;
    private CANSparkMax m_leftDriveFollower;
    private CANSparkMax m_rightDrive;
    private CANSparkMax m_rightDriveFollower;
    private DifferentialDrive m_robotDrive;
    private double leftEncoderPosition;
    private double rightEncoderPosition;
    DifferentialDriveOdometry diffDriveOdometry;
    private Rotation2d robotRotation;
    private double robotRotationDeg;
    

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

        m_leftDrive.getEncoder().setPositionConversionFactor(0.898 * 0.0254);
        m_rightDrive.getEncoder().setPositionConversionFactor(0.898 * 0.0254);

        leftEncoderPosition = 0;
        rightEncoderPosition = 0;
        //#TODO Measure distance between wheel lines
        diffDriveOdometry = new DifferentialDriveOdometry(
            Rotation2d.fromRadians(2*(leftEncoderPosition - rightEncoderPosition)/27/*distance between wheel lines in ____ */),
            m_leftDrive.getEncoder().getPosition(), 
            m_rightDrive.getEncoder().getPosition(),
            new Pose2d(0, 0, new Rotation2d())); 
                

    }

    
    
    public void periodic() 
    {
        
        setEncoderValueL();
        setEncoderValueR();

        Pose2d fieldPose2d;
        robotRotation = Rotation2d.fromRadians(2*(leftEncoderPosition - rightEncoderPosition)/27/*distance between wheel lines in ____ */);
        SmartDashboard.putNumber("Left Encoder Pos", leftEncoderPosition);
        SmartDashboard.putNumber("Right Encoder Pos", rightEncoderPosition);
        diffDriveOdometry.update(robotRotation, leftEncoderPosition, rightEncoderPosition);
        fieldPose2d = diffDriveOdometry.getPoseMeters();
        robotRotationDeg = robotRotation.getDegrees();
        SmartDashboard.putNumber("fieldPose2D X", fieldPose2d.getX());
        SmartDashboard.putNumber("fieldPose2D Y", fieldPose2d.getY());
        SmartDashboard.putNumber("RobotRotationDeg(CW=-)", robotRotationDeg);
        SmartDashboard.putNumber("Odometry X value", diffDriveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y value", diffDriveOdometry.getPoseMeters().getY());


        
    }
    
    public void arcadeDrive(double xSpeed, double zRotation) 
    {
        m_robotDrive.arcadeDrive(xSpeed, zRotation);
    }

    public double getEncoderPosL() 
    {
        double lOdometry = m_leftDrive.getEncoder().getPosition();
        return lOdometry;
    }

    public double getEncoderPosR() 
    {
        double rOdometry = m_rightDrive.getEncoder().getPosition();
        return rOdometry;
    }

    public void setEncoderValueL() 
    {
        leftEncoderPosition = getEncoderPosL();
    }

    public void setEncoderValueR() 
    {
        rightEncoderPosition = getEncoderPosR();
    }

    public void zeroEncoderValueL() 
    {
        m_leftDrive.getEncoder().setPosition(0);
    }

    public void zeroEncoderValueR() 
    {
        m_rightDrive.getEncoder().setPosition(0);
    }

    // Triggered in RobotConainer
    public void zeroEncoderValue() 
    {
        zeroEncoderValueR();
        zeroEncoderValueL();
        System.out.println("Zeroed Encoder Values");
    }
    public Pose2d getPose() 
    {
        Pose2d fielPose2d = diffDriveOdometry.getPoseMeters();
        //Pose2d dif
        return new Pose2d(-fielPose2d.getX(), -fielPose2d.getY(), fielPose2d.getRotation());
    }
    public void resetBot_PosRot()
    {
        diffDriveOdometry.resetPosition(Rotation2d.fromDegrees(robotRotationDeg),
        leftEncoderPosition, rightEncoderPosition, new Pose2d(0, 0, new Rotation2d()));;
        System.out.println("Pos Reset");
    }

    public void zeroOdometry()
    {
        diffDriveOdometry.resetPosition(robotRotation,
        0, 0, new Pose2d(0, 0, new Rotation2d()));;
        System.out.println("Odo Zeroed");
    }

}
