package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{

    private final SparkMax m_ElevatorLeftSpark; 
    private final SparkMax m_ElevatorRightSpark; 
    private final SparkMaxConfig m_ElevatorConfig;
    private final SparkMaxConfig m_ElevatorFollowerConfig;
    private final AbsoluteEncoder m_ElevatorEncoder;

    public ElevatorSubsystem(){

        // Left Elevator Motor 
        m_ElevatorConfig = new SparkMaxConfig();
        m_ElevatorConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);        
        m_ElevatorLeftSpark = new SparkMax(ElevatorConstants.kElevatorLeftCanId, MotorType.kBrushless);
        m_ElevatorLeftSpark.configure(m_ElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right Elevator Motor 
        m_ElevatorFollowerConfig = new SparkMaxConfig();
        m_ElevatorFollowerConfig.apply(m_ElevatorConfig).follow(m_ElevatorLeftSpark).inverted(true);
        m_ElevatorRightSpark = new SparkMax(ElevatorConstants.kElevatorRightCanId, MotorType.kBrushless);   
        m_ElevatorRightSpark.configure(m_ElevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Elevator Encoder
        m_ElevatorEncoder = m_ElevatorLeftSpark.getAbsoluteEncoder();
    }

    public double getPosition() {
        // Right now I'm assuming we'll start by using the left SparkMax motor encoder to determine position, but we might want to add something 
        // like a linear magnetic encoder or string potentiometer for more accuracy. 
        return m_ElevatorEncoder.getPosition();
    }

    public void raise() {
        m_ElevatorLeftSpark.set(ElevatorConstants.kElevatorSpeed);
    }

    public void stop() {
        m_ElevatorLeftSpark.stopMotor();
    }

    public void lower() {        
        m_ElevatorLeftSpark.set(-ElevatorConstants.kElevatorSpeed);
    }

}
