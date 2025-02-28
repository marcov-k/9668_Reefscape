package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkController {
    
    // Network Tables
    NetworkTable NetworkController;
    public DoubleSubscriber leftJoyX;
    public DoubleSubscriber leftJoyY;
    public DoubleSubscriber rightJoyX;
    public DoubleSubscriber rightJoyY;
    public DoubleSubscriber leftTrigger;
    public DoubleSubscriber rightTrigger;
    public BooleanSubscriber aButton;
    public BooleanSubscriber bButton;
    public BooleanSubscriber xButton;
    public BooleanSubscriber yButton;
    public BooleanSubscriber leftBumper;
    public BooleanSubscriber rightBumper;
    public BooleanSubscriber backButton;
    public BooleanSubscriber startButton;
    public BooleanSubscriber leftStickButton;
    public BooleanSubscriber rightStickButton;
    public BooleanSubscriber dpadUp;
    public BooleanSubscriber dpadDown;
    public BooleanSubscriber dpadLeft;
    public BooleanSubscriber dpadRight;


    public void Initialize() {
        NetworkController = NetworkTableInstance.getDefault().getTable("NetworkController");
        leftJoyX = NetworkController.getDoubleTopic("leftJoyX").subscribe(0.00);
        leftJoyY = NetworkController.getDoubleTopic("leftJoyY").subscribe(0.00);
        rightJoyX = NetworkController.getDoubleTopic("rightJoyX").subscribe(0.00);
        rightJoyY = NetworkController.getDoubleTopic("rightJoyY").subscribe(0.00);
        leftTrigger = NetworkController.getDoubleTopic("leftTrigger").subscribe(0.00);
        rightTrigger = NetworkController.getDoubleTopic("rightTrigger").subscribe(0.00);
        aButton = NetworkController.getBooleanTopic("aButton").subscribe(false);
        bButton = NetworkController.getBooleanTopic("bButton").subscribe(false);
        xButton = NetworkController.getBooleanTopic("xButton").subscribe(false);
        yButton = NetworkController.getBooleanTopic("yButton").subscribe(false);
        leftBumper = NetworkController.getBooleanTopic("leftBumper").subscribe(false);
        rightBumper = NetworkController.getBooleanTopic("rightBumper").subscribe(false);
        backButton = NetworkController.getBooleanTopic("backButton").subscribe(false);
        startButton = NetworkController.getBooleanTopic("startButton").subscribe(false);
        leftStickButton = NetworkController.getBooleanTopic("leftStickButton").subscribe(false);
        rightStickButton = NetworkController.getBooleanTopic("rightStickButton").subscribe(false);
        dpadUp = NetworkController.getBooleanTopic("dpadUp").subscribe(false);
        dpadDown = NetworkController.getBooleanTopic("dpadDown").subscribe(false);
        dpadLeft = NetworkController.getBooleanTopic("dpadLeft").subscribe(false);
        dpadRight = NetworkController.getBooleanTopic("dpadRight").subscribe(false);

    }

}
