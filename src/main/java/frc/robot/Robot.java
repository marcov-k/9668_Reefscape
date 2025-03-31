// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.NetworkController;



public class Robot extends TimedRobot {

  // Auto time  
  long elapsedTime;
  long autonomousDuration = 15000;
  long startTime;

  // Drive command variables
  Double strafe;
  Double forward;
  Double rotate;
  boolean fieldRelative;
  boolean rateLimit;
  boolean teleautonomous;
  boolean humandriver;
  Integer elevatorlevel;
  boolean autonomousunfolding;

  // The robot's subsystems
  private final DriveSubsystem swerveDrive = new DriveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final CoralSubsystem coral = new CoralSubsystem();
  private final AlgaeSubsystem algae = new AlgaeSubsystem();

  // PhotonVision 
  PhotonCamera camera = new PhotonCamera("photonvision");
  boolean targetVisible;
  double targetYaw;
  int tagid;
  int bestTarget;
  double largestArea;
  double targetRange;

  // The driver's controller
  private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);

  public class DPadHelper {
    private final XboxController controller;
    private int lastPOV = -1;

    public DPadHelper(XboxController controller) {
        this.controller = controller;
    }

    public void update() {
        lastPOV = controller.getPOV();
    }

    public boolean getDPadUpPressed() {
        int current = controller.getPOV();
        return current == 0 && lastPOV != 0;
    }

    public boolean getDPadDownPressed() {
        int current = controller.getPOV();
        return current == 180 && lastPOV != 180;
    }

    // Call at end of periodic loop to lock in current state
    public void finalizeUpdate() {
        lastPOV = controller.getPOV();
    }
  }

  DPadHelper dPad = new DPadHelper(new XboxController(0));

  NetworkTable GameManager;
  NetworkTableEntry nthumandriver;
  NetworkTableEntry ntfieldrelative;
  NetworkController autoController = new NetworkController();
  

  @Override
  public void robotInit() {    
    swerveDrive.setPose(0,0,180);
    elevator.init();
    nthumandriver = NetworkTableInstance.getDefault().getTable("GameManager").getEntry("HumanDriver");
    ntfieldrelative = NetworkTableInstance.getDefault().getTable( "GameManager").getEntry("FieldRelative");
    autoController.Initialize();
  }

  @Override
  public void robotPeriodic() {
    swerveDrive.periodic();
    elevator.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {  
    // Initially using field relative with rate limits
    elevator.init();
    fieldRelative = false;
    rateLimit = false;    
    nthumandriver.setBoolean(false);
    autonomousunfolding = true;
    algae.init();
    coral.init();
    forward = 0.20;
    strafe = 0.0;
    rotate = 0.0;
    startTime = System.currentTimeMillis();
  }

  @Override
  public void autonomousPeriodic() {

    forward = 0.0;
    strafe = 0.0;
    rotate = 0.0; 

    elapsedTime = System.currentTimeMillis() - startTime;
    
    // Drive forward for two seconds
    if (elapsedTime < 2000) {
      forward = 0.2;
    }
    // For the next 5 seconds rotate until you see a Reef AprilTag, then rotate and drive towards it
    else if (elapsedTime < 7000) {      
      targetVisible = false;
      targetYaw = 0.0;
      targetRange = 0.0;
      largestArea = 0.0;
      var results = camera.getAllUnreadResults();

      if (!results.isEmpty()){
        // Get the most recent frame
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                tagid = target.getFiducialId();
                // If the tag is a reef tag
                if ((tagid > 6 && tagid < 11) || (tagid > 17 && tagid < 21)) {
                  // Find the closest reef tag (takes up the largest area of screen)
                  if (target.getArea() > largestArea) {
                    largestArea = target.getArea();
                    bestTarget = tagid;
                    targetYaw = target.getYaw();
                    targetRange = PhotonUtils.calculateDistanceToTargetMeters(Constants.PhotonVisionConstants.kCameraHeight, Constants.PhotonVisionConstants.kReefAprilTagHeight, Units.degreesToRadians(-30.0), Units.degreesToRadians(target.getPitch()));
                    targetVisible = true;
                  }
                  
                }
            }            
            
        }

      }

      rotate = targetYaw * Constants.DriveConstants.kMaxAngularSpeed * 0.2;
      forward = (targetRange - Constants.PhotonVisionConstants.kReefAprilTagDistance) * 0.2;

    }
    
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);
  }

  @Override
  public void teleopInit() {
    // Initially using field relative with rate limits
    elevator.init();
    fieldRelative = true;
    rateLimit = false;
    teleautonomous = false;
    nthumandriver.setBoolean(true);    
    elevatorlevel = 0;
    swerveDrive.zeroHeading();
  }

  @Override
  public void teleopPeriodic() {
    
    
    // A and Y Button - control elevator
    if (controller.getAButton()) {         
      elevator.lower();  
    } else if (controller.getYButton()) { 
      elevator.raise();   
    } else {
      elevator.stop();
    }

    
    // B and X Button - control coral wrist
    if (controller.getBButton()) {
      coral.wristraise();
    } else if (controller.getXButton()) {
      coral.wristlower();
    } else {
      coral.wriststop();
    }


    // Triggers - control intake and outtake
    if (controller.getRightTriggerAxis() > 0.05) {  
      algae.intake();
      //coral.intake();
    } else if (controller.getLeftTriggerAxis() > 0.05) {
      algae.outtake();
      //coral.outtake();
    } else {
      algae.stop();
      //coral.stop();
    }


    // Bumpers - control Algae Wrist
    if (controller.getRightBumperButton()) {
      algae.wristraise();      
    } else if (controller.getLeftBumperButton()) {
      algae.wristlower();
    } else {
      algae.wriststop();
    }
    
    
    // Back button - Toggles autonomous mode     
    if (controller.getBackButtonPressed()) {
      //teleautonomous = !teleautonomous;  
      swerveDrive.zeroHeading();
    }
    
    // Start button - Toggle field relative and reset heading.
    if (controller.getStartButtonPressed()) {            
      fieldRelative = !fieldRelative;
      
    }

    // Get control values from the controller and apply speed limit and deadband
    strafe = MathUtil.applyDeadband(controller.getLeftX() * OIConstants.kDriverSpeedLimit * elevator.elevatorspeedlimiter, OIConstants.kDriveDeadband);
    forward = MathUtil.applyDeadband(-controller.getLeftY() * OIConstants.kDriverSpeedLimit * elevator.elevatorspeedlimiter, OIConstants.kDriveDeadband);
    rotate = MathUtil.applyDeadband(controller.getRightX() * OIConstants.kDriverRotationLimit, OIConstants.kDriveDeadband);

    // Evaluate whether a driver is driving and publish to networktables
    humandriver = !(strafe == 0 && forward == 0 && rotate == 0);
    nthumandriver.setBoolean(humandriver);

    // If a human isn't currently driving and we're in teleautonomous mode
    //if (teleautonomous && !humandriver) {
      // Use controller values from network tables 
      //strafe = autoController.leftJoyX.get();
      //forward = autoController.leftJoyY.get();
      //rotate = autoController.rightJoyX.get();
    //}
    

    // Send controller values to swerve drive
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);

  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {
     
    // DPad Up and Down - control elevator
    if (dPad.getDPadUpPressed()) {
      elevatorlevel +=1;
      elevator.levelchanged = true;
    }
    else if (dPad.getDPadDownPressed()) {
      elevatorlevel -=1;
      elevator.levelchanged = true;
    }
    dPad.finalizeUpdate();
    elevatorlevel = elevatorlevel % 5;
    elevator.goToLevel(elevatorlevel);    


  }
}