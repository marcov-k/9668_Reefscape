package frc.robot.subsystems;

import frc.robot.Constants.AlgaeConstants;
import edu.wpi.first.networktables.NetworkTableEntry;
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


public class AlgaeSubsystem extends SubsystemBase{

    private final SparkMax m_AlgaeLeftSpark; 
    private final SparkMax m_AlgaeRightSpark;
    private final SparkMax m_AlgaeWristSpark; 
    
    private SparkClosedLoopController AlgaeClosedLoopController;
    private RelativeEncoder encoder;
    
    public boolean unfolded;
    public boolean partiallyunfolded;

    // NetworkTable Entries for Position
    private NetworkTableEntry NTAlgaePosition;
    
    public AlgaeSubsystem(){

        // Algae Intake 
        m_AlgaeLeftSpark = new SparkMax(AlgaeConstants.kAlgaeLeadCanID, MotorType.kBrushless);
        m_AlgaeLeftSpark.configure(AlgaeConstants.AlgaeLead, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        AlgaeConstants.AlgaeFollow.follow(m_AlgaeLeftSpark, true);
        m_AlgaeRightSpark = new SparkMax(AlgaeConstants.kAlgaeFollowCanID, MotorType.kBrushless);
        m_AlgaeRightSpark.configure(AlgaeConstants.AlgaeFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Algae Wrist Motor         
        m_AlgaeWristSpark = new SparkMax(AlgaeConstants.kAlgaeWristCanID, MotorType.kBrushless);   
        m_AlgaeWristSpark.configure(AlgaeConstants.AlgaeWrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // PID Controller
        AlgaeClosedLoopController = m_AlgaeLeftSpark.getClosedLoopController();
    
        // Algae Encoder
        encoder = m_AlgaeWristSpark.getEncoder();

        unfolded = false;
        partiallyunfolded = false;

        
        // Initialize NetworkTable variables
        NetworkTable ElevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
        NTAlgaePosition = ElevatorTable.getEntry("AlgaePosition");
    }

    public double getPosition() {
        // Right now I'm assuming we'll start by using the left SparkMax motor encoder to determine position, but we might want to add something 
        // like a linear magnetic encoder or string potentiometer for more accuracy. 
        return encoder.getPosition();
    }

    public void intake() {
        m_AlgaeLeftSpark.set(-AlgaeConstants.kAlgaeSpeed);
    }

    public void stop() {
        m_AlgaeLeftSpark.stopMotor();
    }

    public void outtake() {        
        m_AlgaeLeftSpark.set(AlgaeConstants.kAlgaeSpeed);
    }

    public void wristraise() {
        NTAlgaePosition.setDouble(encoder.getPosition());
        m_AlgaeWristSpark.set(-AlgaeConstants.kAlgaeWristSpeed);
    }

    public void wriststop() {
        m_AlgaeWristSpark.stopMotor();
    }

    public void wristlower() { 
        NTAlgaePosition.setDouble(encoder.getPosition());       
        m_AlgaeWristSpark.set(AlgaeConstants.kAlgaeWristSpeed);
    }

    public void outtakeprecise() {
        AlgaeClosedLoopController.setReference(2.0,  ControlType.kPosition);            
    }
    
    public void unfold() {
        
        NTAlgaePosition.setDouble(encoder.getPosition());
        if (encoder.getPosition() < 5) {
            wristlower();
        }
        else if (encoder.getPosition() < 10) {
            wristlower();
            partiallyunfolded = true;
        }
        else if (encoder.getPosition() > 35) {
            unfolded = true;
            wriststop();
        }           
    }

    public void init() {
        encoder.setPosition(0);
        unfolded = false;
    }
    

}
