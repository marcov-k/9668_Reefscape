package frc.robot.subsystems;
import frc.robot.Constants.AlgaeConstants;
import frc.utils.Common;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class AlgaeSubsystem extends SubsystemBase{

    private final SparkMax m_AlgaeLeftSpark; 
    private final SparkMax m_AlgaeRightSpark;
    private final SparkMax m_AlgaeWristSpark; 
    private RelativeEncoder encoder;
    public double currentposition;
    private NetworkTableEntry NTAlgaePosition;
    public boolean manualcontrol;
    private double previousp;

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
    
        // Algae Encoder
        encoder = m_AlgaeWristSpark.getEncoder();

        // Initialize NetworkTable variables
        NetworkTable Table = NetworkTableInstance.getDefault().getTable("Algae");
        NTAlgaePosition = Table.getEntry("WristPosition"); }

    public void robotPeriodic() {
        currentposition = encoder.getPosition();
        NTAlgaePosition.setDouble(currentposition);}

    public void teleopPeriodic(boolean AlgaeMode, int elevatorlevel) {
        if (!manualcontrol && AlgaeMode) { 
            // Algae Mode Elevator Levels are 0-Stow, 1-GroundIntake, 2-AlgaeProcessor, 3-L2, 4-L3, 5-Max
            // Algae Wrist Levels are 0-Stowed, 1-Unfolded, 2-Aim for Barge
            if (elevatorlevel == 0)
                goToPosition(AlgaeConstants.algaewristlevels[0]); // Stow
            else if (elevatorlevel == 5)
                goToPosition(AlgaeConstants.algaewristlevels[2]); // Shoot at Barge
            else
                goToPosition(AlgaeConstants.algaewristlevels[1]);}} // AlgaeIntake
            

    public void intake() {
        m_AlgaeLeftSpark.set(-AlgaeConstants.kAlgaeSpeed);}

    public void stop() {
        m_AlgaeLeftSpark.stopMotor();}

    public void outtake() {        
        m_AlgaeLeftSpark.set(AlgaeConstants.kAlgaeSpeed);}

    public void wristraise() {
        manualcontrol = true;
        m_AlgaeWristSpark.set(-AlgaeConstants.kAlgaeWristSpeed);}

    public void wriststop() {
        manualcontrol = true;
        m_AlgaeWristSpark.stopMotor();}

    public void wristlower() {         
        m_AlgaeWristSpark.set(AlgaeConstants.kAlgaeWristSpeed);}

    public void fold() {
        goToPosition(AlgaeConstants.algaewristlevels[0]);}
    
    public void unfold() {
        goToPosition(AlgaeConstants.algaewristlevels[1]);}

    public void init() {
        encoder.setPosition(0);}
    
    private void goToPosition(double targetposition) {
        currentposition = encoder.getPosition();
        double error = (targetposition - currentposition) / Math.max(Math.abs(targetposition), 1.0);  
        error = Common.clamp(error, -1.0, 1.0, 0.01);
        double p = error * 0.6;  // PID - This is proportional
        double d = (p - previousp) * 0.2;
        double speed = p + d; 
        if (Math.abs(targetposition - currentposition) < 0.5) {  // If close enough to target, stop, otherwise set speed
            previousp = 0; 
            m_AlgaeWristSpark.stopMotor();        }
        else {
            previousp = p;
            m_AlgaeWristSpark.set(speed);}}
}
