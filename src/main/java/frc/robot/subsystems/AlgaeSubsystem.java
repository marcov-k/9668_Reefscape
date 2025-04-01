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
import edu.wpi.first.networktables.NetworkTableInstance;


public class AlgaeSubsystem extends SubsystemBase{

    private final SparkMax m_AlgaeLeftSpark; 
    private final SparkMax m_AlgaeRightSpark;
    private final SparkMax m_AlgaeWristSpark; 
    private SparkClosedLoopController AlgaeClosedLoopController;
    private RelativeEncoder encoder;
    public double currentposition;
    private NetworkTableEntry NTAlgaePosition;
    
    public AlgaeSubsystem(){

        // Algae Intake 
        m_AlgaeLeftSpark = new SparkMax(AlgaeConstants.kAlgaeLeadCanID, MotorType.kBrushless);
        m_AlgaeLeftSpark.configure(AlgaeConstants.AlgaeLead, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        AlgaeConstants.AlgaeFollow.follow(m_AlgaeLeftSpark, true);
        m_AlgaeRightSpark = new SparkMax(AlgaeConstants.kAlgaeFollowCanID, MotorType.kBrushless);
        m_AlgaeRightSpark.configure(AlgaeConstants.AlgaeFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Algae Wrist         
        m_AlgaeWristSpark = new SparkMax(AlgaeConstants.kAlgaeWristCanID, MotorType.kBrushless);   
        m_AlgaeWristSpark.configure(AlgaeConstants.AlgaeWrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // PID Controller
        AlgaeClosedLoopController = m_AlgaeWristSpark.getClosedLoopController();
    
        // Algae Encoder
        encoder = m_AlgaeWristSpark.getEncoder();

        // Initialize NetworkTable variables
        NetworkTable ElevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
        NTAlgaePosition = ElevatorTable.getEntry("AlgaePosition"); }

    public void periodic() {
        currentposition = encoder.getPosition();
        NTAlgaePosition.setDouble(currentposition);}

    public void intake() {
        m_AlgaeLeftSpark.set(-AlgaeConstants.kAlgaeSpeed);}

    public void stop() {
        m_AlgaeLeftSpark.stopMotor();}

    public void outtake() {        
        m_AlgaeLeftSpark.set(AlgaeConstants.kAlgaeSpeed);}

    public void wristraise() {
        m_AlgaeWristSpark.set(-AlgaeConstants.kAlgaeWristSpeed);}

    public void wriststop() {
        m_AlgaeWristSpark.stopMotor();}

    public void wristlower() {         
        m_AlgaeWristSpark.set(AlgaeConstants.kAlgaeWristSpeed);}

    public void fold() {
        AlgaeClosedLoopController.setReference(0.0,  ControlType.kPosition);}
    
    public void unfold() {
        AlgaeClosedLoopController.setReference(35, ControlType.kPosition);}

    public void init() {
        encoder.setPosition(0);}
    

}
