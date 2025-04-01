package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

public class ElevatorSubsystem extends SubsystemBase{

    private final SparkFlex m_ElevatorLeftSpark; 
    private final SparkFlex m_ElevatorRightSpark;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
    private NetworkTableEntry NTElevatorPosition;
    private NetworkTableEntry NTElevatorLevel;
    private double currentspeed;
    private double currentposition;
    public double elevatorspeedlimiter;
    public boolean motorrunning;
    private DigitalInput ElevatorLimitSwitch;
    private boolean isLimitPressed;
    private boolean wasLimitPressedLastTime;
    public int level;

    public ElevatorSubsystem(){

        // Left Elevator Motor 
        m_ElevatorLeftSpark = new SparkFlex(ElevatorConstants.kElevatorLeftCanId, MotorType.kBrushless);
        m_ElevatorLeftSpark.configure(ElevatorConstants.leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right Elevator Motor  
        ElevatorConstants.followConfig.follow(m_ElevatorLeftSpark, true);       
        m_ElevatorRightSpark = new SparkFlex(ElevatorConstants.kElevatorRightCanId, MotorType.kBrushless);   
        m_ElevatorRightSpark.configure(ElevatorConstants.followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // PID Controller
        closedLoopController = m_ElevatorLeftSpark.getClosedLoopController();

        // Elevator Encoder
        encoder = m_ElevatorLeftSpark.getEncoder();

        // Elevator Limit Switch returns true when open, false when closed
        ElevatorLimitSwitch = new DigitalInput(0);

        // Initialize NetworkTable variables
        NetworkTable ElevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");
        NTElevatorPosition = ElevatorTable.getEntry("Position");
        NTElevatorLevel = ElevatorTable.getEntry("Level");
        motorrunning = false;
        wasLimitPressedLastTime = false;
        level = 0;
    }

    public void init() {
        // Configure right Elevator Motor to follow left just in case this was missed at startup        
        ElevatorConstants.followConfig.follow(m_ElevatorLeftSpark, true);
        m_ElevatorRightSpark.configure(ElevatorConstants.followConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }


    private double scaledSpeedToTop() {
        return ElevatorConstants.kElevatorSpeed * Math.min(100,(ElevatorConstants.kHighestLevel - currentposition))/100; }
    
    private double scaledSpeedToBottom() {
        return -ElevatorConstants.kElevatorSpeed * Math.min(100, currentposition)/100; }

    public void periodic() {
        currentposition = encoder.getPosition(); 
        NTElevatorPosition.setDouble(currentposition);
        NTElevatorLevel.setInteger(level);
        // Reset encoder position to zero when limit switch is triggered
        isLimitPressed = !ElevatorLimitSwitch.get();
        if (isLimitPressed && !wasLimitPressedLastTime) {
            encoder.setPosition(0.00);}
        wasLimitPressedLastTime = isLimitPressed;
        // Used to limit swerve drive speed based on elevator height to prevent tipping with a higher center of gravity
        elevatorspeedlimiter = (Constants.ElevatorConstants.kHighestLevel + 50 - currentposition) / ( Constants.ElevatorConstants.kHighestLevel + 50); }

    public void raise() {
        if (currentposition < ElevatorConstants.kHighestLevel) {
            currentspeed = scaledSpeedToTop();
            m_ElevatorLeftSpark.set(currentspeed);}
        else {
            m_ElevatorLeftSpark.stopMotor();}
        motorrunning = true; }

    public void lower() {        
        if (currentposition > ElevatorConstants.kLowestLevel) {
            currentspeed = scaledSpeedToBottom();
            m_ElevatorLeftSpark.set(currentspeed);}
        else {
            m_ElevatorLeftSpark.stopMotor();}
        motorrunning = true; }

    public void stop() {
        m_ElevatorLeftSpark.stopMotor(); }

    public void goToCoralLevel(int level) {
        if (level < 0 || level >= ElevatorConstants.corallevels.length) return;
        closedLoopController.setReference(ElevatorConstants.corallevels[level], ControlType.kPosition);
        motorrunning = false; }

    public void goToAlgaeLevel(int level) {
        //if (level < 0 || level >= ElevatorConstants.algaelevels.length) return;
        closedLoopController.setReference(ElevatorConstants.algaelevels[level], ControlType.kPosition);
        motorrunning = false; }

}
