package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm 
{
    private CANSparkMax m_arm;
    
    public Arm()
    {
        m_arm = new CANSparkMax(5, MotorType.kBrushless);
        m_arm.setIdleMode(IdleMode.kBrake);
        m_arm.setSmartCurrentLimit(40,15);
        //#TODO set nums
        m_arm.getEncoder().setPositionConversionFactor(360/75);
        m_arm.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_arm.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_arm.setSoftLimit(SoftLimitDirection.kForward, 0);//set
        m_arm.setSoftLimit(SoftLimitDirection.kReverse, 0);//set
        SparkMaxPIDController pidController = m_arm.getPIDController();
        pidController.setP(0.01);
        pidController.getI(0);
        pidController.setD(0);
        pidController.setFF(0);
        pidController.setOutputRange(-0.25, 0.25);
        m_arm.getEncoder().setPosition(0);
    }

    public void periodic()
    {
        SmartDashboard.putNumber("ArmJointPosition", m_arm.getEncoder().getPosition());
    }

    public void setArm(double pos)
    {
        m_arm.getPIDController().setReference(pos, com.revrobotics.CANSparkMax.ControlType.kPosition);
    }



    

}
