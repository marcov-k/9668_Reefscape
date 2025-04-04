package frc.robot.subsystems;
import frc.robot.Constants.CoralConstants;
import frc.utils.Common;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;



public class CoralSubsystem extends SubsystemBase{

    private final SparkMax m_CoralLeftSpark; 
    private final SparkMax m_CoralWristSpark; 
    
    private RelativeEncoder encoder;
    //private NetworkTableEntry NTCoralPosition;
    public double currentposition;
    public boolean manualcontrol;
    private double previousp;
    private boolean L4Scoring;
    private double desiredposition;

    public CoralSubsystem(){

        // Coral Motor 
        m_CoralLeftSpark = new SparkMax(CoralConstants.kCoralCanID, MotorType.kBrushless);
        m_CoralLeftSpark.configure(CoralConstants.coral, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Wrist Motor         
        m_CoralWristSpark = new SparkMax(CoralConstants.kWristCanID, MotorType.kBrushless);   
        m_CoralWristSpark.configure(CoralConstants.wrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);        

        // Coral Encoder
        encoder = m_CoralWristSpark.getEncoder();
    }

    public void robotPeriodic() {
        currentposition = encoder.getPosition();
    }

    public void teleopPeriodic(boolean CoralMode, int elevatorlevel) {
        if (!manualcontrol && CoralMode) {
            L4Scoring = false;
            // Coral Mode Elevator Levels are 0-Stow, 1-CoralIntake, 2-L1, 3-L2, 4-L3, 5-L4
            // Coral Wrist Levels are 0-Stow, 1-CoralIntake, 2-L1-3Score, 3-L4Score
            if (elevatorlevel == 0){
                goToPosition(CoralConstants.coralwristlevels[0]);} // Stow
            else if (elevatorlevel == 1){
                goToPosition(CoralConstants.coralwristlevels[1]);} // CoralIntake
            else if (elevatorlevel == 5){
                L4Scoring = true;
                goToPosition(CoralConstants.coralwristlevels[3]);} // L4 Score
            else {
                goToPosition(CoralConstants.coralwristlevels[2]);}} // L1-L3 Score
    } 
        
    public void autonomousPeriodic() {
        goToPosition(CoralConstants.coralwristlevels[2]);}
         
    public void intake() {
        m_CoralLeftSpark.set(-CoralConstants.kCoralSpeed);}

    public void stop() {
        m_CoralLeftSpark.stopMotor();}

    public void outtake() { 
        m_CoralLeftSpark.set(CoralConstants.kCoralSpeed);
        if (L4Scoring){
            goToPosition(14);}} // On an L4 Score, shoot while lifting the wrist from 30 to 14 to help knock it on there

    public void wristraise() {  
        manualcontrol = true;
        m_CoralWristSpark.set(-CoralConstants.kCoralWristSpeed);
        desiredposition = currentposition;}

    public void wriststop() {       
        goToPosition(desiredposition);}

    public void wristlower() {                
        manualcontrol = true;
        m_CoralWristSpark.set(CoralConstants.kCoralWristSpeed);
        desiredposition = currentposition;}

    public void fold() {
        goToPosition(CoralConstants.coralwristlevels[0]);}
    
    public void unfold(int elevatorlevel) {
        L4Scoring = false;
        // Coral Mode Elevator Levels are 0-Stow, 1-CoralIntake, 2-L1, 3-L2, 4-L3, 5-L4
        // Coral Wrist Levels are 0-Stow, 1-CoralIntake, 2-L1-3Score, 3-L4Score
        if (elevatorlevel == 0){
            goToPosition(CoralConstants.coralwristlevels[0]);} // Stow
        else if (elevatorlevel == 1){
            goToPosition(CoralConstants.coralwristlevels[1]);} // CoralIntake
        else if (elevatorlevel == 5){
            L4Scoring = true;
            goToPosition(CoralConstants.coralwristlevels[3]);} // L4 Score
        else {
            goToPosition(CoralConstants.coralwristlevels[2]);}} // L1-L3 Score

    public void scoringpose() {
        goToPosition(CoralConstants.coralwristlevels[2]);} // L1-L3 Score

    public void intakepose() {
        goToPosition(CoralConstants.coralwristlevels[1]);} // CoralIntake   

    public void init() {
        encoder.setPosition(0);
        desiredposition = 0;}

    private void goToPosition(double targetposition) {
        currentposition = encoder.getPosition();
        double error = (targetposition - currentposition) / Math.max(Math.abs(targetposition), 1.0);   
        error = Common.clamp(error, -1.0, 1.0, 0.01);
        double p = error * 0.3;  // PID - This is proportional
        double d = (p - previousp) * 0.05;
        double speed = p + d; 
        if (Math.abs(targetposition - currentposition) < 0.5) {  // If close enough to target, stop, otherwise set speed
            previousp = 0; 
            m_CoralWristSpark.stopMotor();        }
        else {
            previousp = p;
            m_CoralWristSpark.set(speed);}}

}
