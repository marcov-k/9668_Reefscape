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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class CoralSubsystem extends SubsystemBase{

    private final SparkMax m_CoralLeftSpark; 
    private final SparkMax m_CoralWristSpark; 
    
    private SparkClosedLoopController CoralClosedLoopController;
    private RelativeEncoder encoder;

    
    // NetworkTable Entries for Position
    private NetworkTableEntry NTCoralPosition;


    public boolean unfolded;

    public CoralSubsystem(){

        // Coral Motor 
        m_CoralLeftSpark = new SparkMax(CoralConstants.kCoralCanID, MotorType.kBrushless);
        m_CoralLeftSpark.configure(CoralConstants.coral, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Wrist Motor         
        m_CoralWristSpark = new SparkMax(CoralConstants.kWristCanID, MotorType.kBrushless);   
        m_CoralWristSpark.configure(CoralConstants.wrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // PID Controller
        CoralClosedLoopController = m_CoralWristSpark.getClosedLoopController();
    
        // Coral Encoder
        encoder = m_CoralWristSpark.getEncoder();

        unfolded = false;

        
        // Initialize NetworkTable variables
        NetworkTable ElevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");

        NTCoralPosition = ElevatorTable.getEntry("CoralPosition");
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
        NTCoralPosition.setDouble(encoder.getPosition());
        m_CoralWristSpark.set(-CoralConstants.kCoralWristSpeed);
    }

    public void wriststop() {
        m_CoralWristSpark.stopMotor();
    }

    public void wristlower() {        
        NTCoralPosition.setDouble(encoder.getPosition());
        m_CoralWristSpark.set(CoralConstants.kCoralWristSpeed);
    }

    public void fold() {
        CoralClosedLoopController.setReference(0.0,  ControlType.kPosition);            
    }
    
    public void unfold() {
        CoralClosedLoopController.setReference(20, ControlType.kPosition);
    }

    public void auto() {
        CoralClosedLoopController.setReference(20,  ControlType.kPosition); 
    }

    public void init() {
        encoder.setPosition(0);
        unfolded = false;
    }

}
