package frc.robot.subsystems;
import frc.robot.Constants.AlgaeConstants;
import frc.utils.Common;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;


public class AlgaeSubsystem extends SubsystemBase{

    private final SparkMax m_AlgaeLeftSpark; 
    private final SparkMax m_AlgaeRightSpark;
    private final SparkMax m_AlgaeWristSpark;
    private final long demoCutoffTime;
    private final long demoPauseTime;
    private final long shootTime;
    private RelativeEncoder encoder;
    public double currentposition;
    private double desiredposition;
    // private NetworkTableEntry NTAlgaePosition;
    public boolean manualcontrol;
    private double previousp;
    private long currentTime;
    public boolean doingDemo;
    private boolean pickingUp;
    private boolean pausing;
    private boolean shooting;
    private long targetTime;
    private DigitalInput algaeSensor;

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

        // Demo Functionality
        demoCutoffTime = 3000;
        demoPauseTime = 4000;
        shootTime = 1000;
        algaeSensor = new DigitalInput(0);

        // Initialize NetworkTable variables
        // NetworkTable Table = NetworkTableInstance.getDefault().getTable("Algae");
        // NTAlgaePosition = Table.getEntry("WristPosition"); 
        }

    public void robotPeriodic() {
        currentposition = encoder.getPosition();
        // NTAlgaePosition.setDouble(currentposition);
        }

    public void teleopPeriodic() {
        if (!manualcontrol) 
        { 
            if (doingDemo)
            {
                handleBallDemo();
            }
        }
    } 

    public void ballDemo()
    {
        if (doingDemo && !manualcontrol)
        {
            stop();
            resetDemo();
        }
        else
        {
            manualcontrol = false;
            resetDemo();
            doingDemo = true;
            targetTime = System.currentTimeMillis() + demoCutoffTime;
            pickingUp = true;
        }
    }

    public void handleBallDemo()
    {
        if (pickingUp)
        {
            pickUpBall();
        }
        else if (pausing)
        {
            demoPause();
        }
        else if (shooting)
        {
            shootBall();
        }
    }

    public void pickUpBall()
    {
        currentTime = System.currentTimeMillis();
        if (algaeSensor.get())
        {
            pickingUp = false;
            targetTime = currentTime + demoPauseTime;
            pausing = true;
            stop();
        }
        else if (currentTime < targetTime)
        {
            intake();
        }
        else
        {
            resetDemo();
            stop();
        }
    }

    public void demoPause()
    {
        currentTime = System.currentTimeMillis();
        if (currentTime >= targetTime)
        {
            pausing = false;
            targetTime = currentTime + shootTime;
            shooting = true;
        }
    }

    public void shootBall()
    {
        currentTime = System.currentTimeMillis();
        if (currentTime < targetTime)
        {
            outtake();
        }
        else
        {
            stop();
            resetDemo();
        }
    }

    public void resetDemo()
    {
        pickingUp = false;
        pausing = false;
        shooting = false;
        doingDemo = false;
    }

    public void intake() {
        m_AlgaeLeftSpark.set(AlgaeConstants.kAlgaeSpeed);}

    public void stop() {
        m_AlgaeLeftSpark.stopMotor();}

    public void outtake() {        
        m_AlgaeLeftSpark.set(-AlgaeConstants.kAlgaeSpeed);}

    public void wristraise() {
        manualcontrol = true;
        m_AlgaeWristSpark.set(-AlgaeConstants.kAlgaeWristSpeed);
        desiredposition = currentposition;
    }

    public void wriststop() {
        goToPosition(desiredposition);
        //m_AlgaeWristSpark.stopMotor();
    }

    public void wristlower() {         
        manualcontrol = true;
        m_AlgaeWristSpark.set(AlgaeConstants.kAlgaeWristSpeed);
        desiredposition = currentposition;
    }

    public void fold() {
        goToPosition(AlgaeConstants.algaewristlevels[0]);}
    
    public void unfold() {
        goToPosition(AlgaeConstants.algaewristlevels[1]);}

    public void init() {
        encoder.setPosition(0);
        currentTime = 0;
        targetTime = 0;
        doingDemo = false;
        pickingUp = false;
        pausing = false;
        shooting = false;
    }
    
    private void goToPosition(double targetposition) {
        currentposition = encoder.getPosition();
        double error = (targetposition - currentposition) / Math.max(Math.abs(targetposition), 1.0);  
        error = Common.clamp(error, -1.0, 1.0, 0.01);
        double p = error * 0.15;  // PID - This is proportional
        double d = (p - previousp) * 0.0;
        double speed = p + d; 
        if (Math.abs(targetposition - currentposition) < 0.5) {  // If close enough to target, stop, otherwise set speed
            previousp = 0; 
            m_AlgaeWristSpark.stopMotor();        }
        else {
            previousp = p;
            m_AlgaeWristSpark.set(speed);}}
}
