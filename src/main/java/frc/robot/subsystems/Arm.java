package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm {
    private CANSparkMax m_arm;

    public Arm()
    {
        m_arm = new CANSparkMax(5, MotorType.kBrushless);

    }

    

}
