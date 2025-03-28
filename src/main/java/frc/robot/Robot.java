// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
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
    forward = 0.2;
  }

  @Override
  public void autonomousPeriodic() {

    if (autonomousunfolding){
      algae.unfold();
      // if (algae.partiallyunfolded) {
        //coral.unfold();
      // }

      if (algae.unfolded) {
        forward = 0.0;
        autonomousunfolding = false; 
      }
      swerveDrive.drive(forward, 0, 0, fieldRelative, rateLimit);     
    }

    // Get control values from network tables
    //strafe = MathUtil.applyDeadband(autoController.leftJoyX.get(), OIConstants.kDriveDeadband);
    //forward = MathUtil.applyDeadband(autoController.leftJoyY.get(), OIConstants.kDriveDeadband);
    //rotate = MathUtil.applyDeadband(autoController.rightJoyX.get(), OIConstants.kDriveDeadband);

    // Send control values to swerve drive
    //swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);

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