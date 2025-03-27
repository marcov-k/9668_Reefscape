package frc.robot.subsystems;

import frc.robot.Constants.CoralConstants;


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
    private final SparkMax m_CoralWristSpark; 
    
    private SparkClosedLoopController coralClosedLoopController;
    private RelativeEncoder encoder;
    

    public boolean unfolded;

    public CoralSubsystem(){

        // Coral Motor 
        m_CoralLeftSpark = new SparkMax(CoralConstants.kCoralCanID, MotorType.kBrushless);
        m_CoralLeftSpark.configure(CoralConstants.coral, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Wrist Motor         
        m_CoralWristSpark = new SparkMax(CoralConstants.kWristCanID, MotorType.kBrushless);   
        m_CoralWristSpark.configure(CoralConstants.wrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // PID Controller
        coralClosedLoopController = m_CoralLeftSpark.getClosedLoopController();
    
        // Coral Encoder
        encoder = m_CoralLeftSpark.getEncoder();

        unfolded = false;
    }

    public double getPosition() {
        // Right now I'm assuming we'll start by using the left SparkMax motor encoder to determine position, but we might want to add something 
        // like a linear magnetic encoder or string potentiometer for more accuracy. 
        return encoder.getPosition();
    }

    public void intake() {
        m_CoralLeftSpark.set(-CoralConstants.kCoralSpeed);
    }

    public void stop() {
        m_CoralLeftSpark.stopMotor();
    }

    public void outtake() {        
        m_CoralLeftSpark.set(CoralConstants.kCoralSpeed);
    }

    public void wristraise() {
        m_CoralWristSpark.set(-CoralConstants.kCoralWristSpeed);
    }

    public void wriststop() {
        m_CoralWristSpark.stopMotor();
    }

    public void wristlower() {        
        m_CoralWristSpark.set(CoralConstants.kCoralWristSpeed);
    }

    public void outtakeprecise() {
        coralClosedLoopController.setReference(2.0,  ControlType.kPosition);            
    }
    
    public void unfold() {
        if (encoder.getPosition() < 15.0) {
            wristlower();
        }
        else {
            wriststop();
            unfolded = true;
        }           
    }

}
