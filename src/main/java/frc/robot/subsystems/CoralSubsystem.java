package frc.robot.subsystems;

import frc.robot.Constants.CoralConstants;
import frc.robot.Configs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

public class CoralSubsystem extends SubsystemBase{

    private final SparkMax m_CoralLeftSpark; 
    private final SparkMax m_CoralRightSpark; 
    
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
    

    public CoralSubsystem(){

        // Left Coral Motor 
        m_CoralLeftSpark = new SparkMax(CoralConstants.kCoralLeftCanId, MotorType.kBrushless);
        m_CoralLeftSpark.configure(Configs.CoralMotor.leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right Coral Motor         
        Configs.CoralMotor.followConfig.follow(m_CoralLeftSpark);
        m_CoralRightSpark = new SparkMax(CoralConstants.kCoralRightCanId, MotorType.kBrushless);   
        m_CoralRightSpark.configure(Configs.CoralMotor.followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // PID Controller
        closedLoopController = m_CoralLeftSpark.getClosedLoopController();

        // Coral Encoder
        encoder = m_CoralLeftSpark.getEncoder();
    }

    public double getPosition() {
        // Right now I'm assuming we'll start by using the left SparkMax motor encoder to determine position, but we might want to add something 
        // like a linear magnetic encoder or string potentiometer for more accuracy. 
        return encoder.getPosition();
    }

    public void intake() {
        m_CoralLeftSpark.set(CoralConstants.kCoralSpeed);
    }

    public void stop() {
        m_CoralLeftSpark.stopMotor();
    }

    public void outtake() {        
        m_CoralLeftSpark.set(-CoralConstants.kCoralSpeed);
    }

    public void outtakeprecise() {
        closedLoopController.setReference(2.0,  ControlType.kPosition);            
    }
    

}
