// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;




public class Robot extends TimedRobot {

  // Drive command variables
  Double strafe;
  Double forward;
  Double rotate;
  boolean fieldRelative;
  boolean rateLimit;
  boolean teleautonomous;
  boolean humandriver;

  // The robot's subsystems
  private final DriveSubsystem swerveDrive = new DriveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();

  // The driver's controller
  private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);
  
  // Network Tables
  NetworkTable NetworkController;
  DoubleSubscriber networkcontroller_leftJoyX;
  DoubleSubscriber networkcontroller_leftJoyY;
  DoubleSubscriber networkcontroller_rightJoyX;
  DoubleSubscriber networkcontroller_rightJoyY;
  DoubleSubscriber networkcontroller_leftTrigger;
  DoubleSubscriber networkcontroller_rightTrigger;
  BooleanSubscriber networkcontroller_aButton;
  BooleanSubscriber networkcontroller_bButton;
  BooleanSubscriber networkcontroller_xButton;
  BooleanSubscriber networkcontroller_yButton;
  BooleanSubscriber networkcontroller_leftBumper;
  BooleanSubscriber networkcontroller_rightBumper;
  BooleanSubscriber networkcontroller_backButton;
  BooleanSubscriber networkcontroller_startButton;
  BooleanSubscriber networkcontroller_leftStickButton;
  BooleanSubscriber networkcontroller_rightStickButton;
  BooleanSubscriber networkcontroller_dpadUp;
  BooleanSubscriber networkcontroller_dpadDown;
  BooleanSubscriber networkcontroller_dpadLeft;
  BooleanSubscriber networkcontroller_dpadRight;
  NetworkTable GameManager;
  NetworkTableEntry nthumandriver;
  


  @Override
  public void robotInit() {
    NetworkTableInstance.getDefault().startServer();
    NetworkController = NetworkTableInstance.getDefault().getTable("NetworkController");
    networkcontroller_leftJoyX = NetworkController.getDoubleTopic("leftJoyX").subscribe(0.00);
    networkcontroller_leftJoyY = NetworkController.getDoubleTopic("leftJoyY").subscribe(0.00);
    networkcontroller_rightJoyX = NetworkController.getDoubleTopic("rightJoyX").subscribe(0.00);
    networkcontroller_rightJoyY = NetworkController.getDoubleTopic("rightJoyY").subscribe(0.00);
    networkcontroller_leftTrigger = NetworkController.getDoubleTopic("leftTrigger").subscribe(0.00);
    networkcontroller_rightTrigger = NetworkController.getDoubleTopic("rightTrigger").subscribe(0.00);
    networkcontroller_aButton = NetworkController.getBooleanTopic("aButton").subscribe(false);
    networkcontroller_bButton = NetworkController.getBooleanTopic("bButton").subscribe(false);
    networkcontroller_xButton = NetworkController.getBooleanTopic("xButton").subscribe(false);
    networkcontroller_yButton = NetworkController.getBooleanTopic("yButton").subscribe(false);
    networkcontroller_leftBumper = NetworkController.getBooleanTopic("leftBumper").subscribe(false);
    networkcontroller_rightBumper = NetworkController.getBooleanTopic("rightBumper").subscribe(false);
    networkcontroller_backButton = NetworkController.getBooleanTopic("backButton").subscribe(false);
    networkcontroller_startButton = NetworkController.getBooleanTopic("startButton").subscribe(false);
    networkcontroller_leftStickButton = NetworkController.getBooleanTopic("leftStickButton").subscribe(false);
    networkcontroller_rightStickButton = NetworkController.getBooleanTopic("rightStickButton").subscribe(false);
    networkcontroller_dpadUp = NetworkController.getBooleanTopic("dpadUp").subscribe(false);
    networkcontroller_dpadDown = NetworkController.getBooleanTopic("dpadDown").subscribe(false);
    networkcontroller_dpadLeft = NetworkController.getBooleanTopic("dpadLeft").subscribe(false);
    networkcontroller_dpadRight = NetworkController.getBooleanTopic("dpadRight").subscribe(false);
    swerveDrive.setPose(0,5,0);
    elevator.goToLevel(0);
    GameManager = NetworkTableInstance.getDefault().getTable("GameManager");
    nthumandriver = GameManager.getEntry("HumanDriver");
  }

  @Override
  public void robotPeriodic() {
    swerveDrive.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {  
    // Initially using field relative with rate limits
    fieldRelative = true;
    rateLimit = false;    
    nthumandriver.setBoolean(false);
    
  }

  @Override
  public void autonomousPeriodic() {

    // Get control values from network tables
    strafe = MathUtil.applyDeadband(networkcontroller_leftJoyX.get(), OIConstants.kDriveDeadband);
    forward = MathUtil.applyDeadband(networkcontroller_leftJoyY.get(), OIConstants.kDriveDeadband);
    rotate = MathUtil.applyDeadband(networkcontroller_rightJoyX.get(), OIConstants.kDriveDeadband);

    // Send control values to swerve drive
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);

  }

  @Override
  public void teleopInit() {
    // Initially using field relative with rate limits
    fieldRelative = true;
    rateLimit = true;
    teleautonomous = false;
    nthumandriver.setBoolean(true);    
  }

  @Override
  public void teleopPeriodic() {
    
    
    // Bumpers to raise or lower elevator (For now)
    if (controller.getRightBumperButton() ) {
      elevator.raise();
    }
    else if (controller.getLeftBumperButton()) {
      elevator.lower();      
    }
    else {
      elevator.stop();      
    }

    // X button - sets wheels in an X formation
    if (controller.getXButtonPressed() ) {
      swerveDrive.setX();
    }

    // Y button - Resets heading and sets pose to 0,5
    if (controller.getYButtonPressed() ) {
      swerveDrive.zeroHeading();      
      swerveDrive.setPose(0,5,0);
    }
    
    // Back button - Toggles field relative    
    if (controller.getBackButtonPressed()) {
      fieldRelative = !fieldRelative;      
    }
    
    // Start button - Toggles autonomous mode
    if (controller.getStartButtonPressed()) {      
      teleautonomous = !teleautonomous;
    }

    // Get control values from the controller and apply speed limit and deadband
    strafe = MathUtil.applyDeadband(controller.getLeftX() * OIConstants.kDriverSpeedLimit, OIConstants.kDriveDeadband);
    forward = MathUtil.applyDeadband(-controller.getLeftY() * OIConstants.kDriverSpeedLimit, OIConstants.kDriveDeadband);
    rotate = MathUtil.applyDeadband(controller.getRightX() * OIConstants.kDriverRotationLimit, OIConstants.kDriveDeadband);

    // Evaluate whether a driver is driving and publish to networktables
    humandriver = !(strafe == 0 && forward == 0 && rotate == 0);
    nthumandriver.setBoolean(humandriver);

    // If a human isn't currently driving and we're in teleautonomous mode
    if (teleautonomous && !humandriver) {
      // Use controller values from network tables 
      strafe = networkcontroller_leftJoyX.get();
      forward = networkcontroller_leftJoyY.get();
      rotate = networkcontroller_rightJoyX.get();
    }
    

    // Send controller values to swerve drive
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);

  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}