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
    private NetworkTableEntry NTCoralPosition;
    public double currentposition;

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
        
        // Initialize NetworkTable variables
        NetworkTable ElevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
        NTCoralPosition = ElevatorTable.getEntry("CoralPosition"); }

    public void periodic() {
        currentposition = encoder.getPosition();
        NTCoralPosition.setDouble(currentposition);}

    public void intake() {
        m_CoralLeftSpark.set(-CoralConstants.kCoralSpeed);}

    public void stop() {
        m_CoralLeftSpark.stopMotor();}

    public void outtake() {        
        m_CoralLeftSpark.set(CoralConstants.kCoralSpeed);}

    public void wristraise() {  
        m_CoralWristSpark.set(-CoralConstants.kCoralWristSpeed);}

    public void wriststop() {
        m_CoralWristSpark.stopMotor();}

    public void wristlower() {                
        m_CoralWristSpark.set(CoralConstants.kCoralWristSpeed);}

    public void fold() {
        CoralClosedLoopController.setReference(0.0,  ControlType.kPosition);}
    
    public void unfold() {
        CoralClosedLoopController.setReference(20, ControlType.kPosition);}

    public void scoringpose() {
        CoralClosedLoopController.setReference(30, ControlType.kPosition);}

    public void intakepose() {
        CoralClosedLoopController.setReference(10, ControlType.kPosition);}    

    public void auto() {
        CoralClosedLoopController.setReference(20,  ControlType.kPosition); }

    public void init() {
        encoder.setPosition(0);}

}
