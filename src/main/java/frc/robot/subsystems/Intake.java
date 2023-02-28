package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase
{
    private CANSparkMax intake;

    public Intake()
    {
        intake = new CANSparkMax(6, MotorType.kBrushless);
        intake.setIdleMode(IdleMode.kCoast);

    }
    //sets motor speed
    public void setIntake(double value)
    {
        intake.set(value);
    }



}
