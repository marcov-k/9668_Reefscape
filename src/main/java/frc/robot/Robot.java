// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DPadHelper;
import frc.utils.Common;



public class Robot extends TimedRobot {

  // Time  
  long elapsedTime;
  long startTime;
  long algaeTime;
  long coralTime;

  // Swerve Drive 
  Double strafe;
  Double forward;
  Double rotate;
  boolean fieldRelative;
  boolean rateLimit;

  // Elevator 
  Integer elevatorlevel;
  boolean CoralMode;

  // Subsystems
  private final DriveSubsystem swerveDrive = new DriveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final CoralSubsystem coral = new CoralSubsystem();
  private final AlgaeSubsystem algae = new AlgaeSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();

  // PhotonVision 
  PhotonCamera camera;
  boolean targetVisible;  
  int tagid;
  int bestTarget;
  double largestArea;
  Transform3d targetTransform;
  Translation3d targetTranslation;
  Rotation3d targetRotation;
  double targetForwardDistance;
  double targetStrafeDistance;
  double targetYaw;
  double targetPitch;
  boolean aligned;
  boolean slowmode;
  double slowspeedlimit;

  // Auto
  boolean shootingAlgae;

  // Controller
  private final XboxController controller = new XboxController(OIConstants.kDriverControllerPort);
  DPadHelper dPad = new DPadHelper(controller);


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
  
  /* ROBOT INIT */
  @Override
  public void robotInit() { 
    swerveDrive.zeroHeading();   
    swerveDrive.setPose(0,0,180);
    elevator.init();
    vision.init();    
  }

  /* ROBOT PERIODIC */
  @Override
  public void robotPeriodic() {
    swerveDrive.periodic();
    elevator.robotPeriodic();
    coral.robotPeriodic();
    algae.robotPeriodic();
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

  /* AUTONOMOUS INIT */
  @Override
  public void autonomousInit() {  
    elevator.init();
    algae.init();
    coral.init();
    fieldRelative = false;
    rateLimit = false;    
    startTime = System.currentTimeMillis();
    aligned = false;
    coral.manualcontrol = false;
    coralTime = 0;
    algaeTime = 0;
  }

  /* AUTONOMOUS PERIODIC */
  @Override
  public void autonomousPeriodic() {
    // Initialize Variables 
    elapsedTime = System.currentTimeMillis() - startTime;
    forward = 0.0;
    strafe = 0.0;
    rotate = 0.0; 
    
    elevator.goToCoralLevel(3);
    coral.scoringpose();
    // Drive forward for two seconds
    if (elapsedTime < 2000) {
      forward = 0.1; }
    // For the next 5 seconds rotate until you see a Reef AprilTag, then rotate and drive towards it
    else if (elapsedTime < 15000 && !aligned) {
      vision.getDirectionsToTarget();
      if (vision.targetVisible()){ // If a target is visible
        forward = vision.forward;
        strafe = vision.strafe;
        rotate = vision.rotate; } 
      else {
        forward = 0.0;
        strafe = 0.0;
        rotate = 0.05; }// Rotate until we see a target      
      aligned = vision.onTarget();
      if (aligned) {
        coralTime = System.currentTimeMillis();
      }
    }
    else if (aligned){
      long elapsedShootTime = System.currentTimeMillis() - coralTime;
      if (elapsedShootTime > 3000 && elapsedShootTime < 3500) 
        coral.outtake(); 
      else
        coral.stop();
        coral.fold(); }
    
    coral.autonomousPeriodic();
    elevator.autonomousPeriodic();
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

  /* TELEOP INIT */
  @Override
  public void teleopInit() {    
    elevator.init();
    fieldRelative = true;
    rateLimit = false;
    CoralMode = true;  
    elevator.manualcontrol = true;
    coral.manualcontrol = true;
    algae.manualcontrol = true;
    slowmode = false;
    slowspeedlimit = 1.0;
  }

  /* TELEOP PERIODIC */
  @Override
  public void teleopPeriodic() {
    
    // DPad Left - Select Coral Mode 
    if (dPad.getDPadLeftPressed()) {
      CoralMode = true;  
      coral.intakepose();
      elevator.goToCoralLevel(1);
      elevator.level = 1;
      elevator.manualcontrol = false;
      algae.manualcontrol = false;
      coral.manualcontrol = false;
      }
    // DPad Right - Select Algae Mode 
    else if (dPad.getDPadRightPressed()) {
      CoralMode = false; 
      elevator.manualcontrol = false;
      algae.manualcontrol = false;
      coral.manualcontrol = false;
      }

    // DPad Up - Go up a level
    if (dPad.getDPadUpPressed()) {      
      elevator.level +=1;
      elevator.level = Common.clamp(elevator.level, 0, 5);               
      elevator.manualcontrol = false;
      algae.manualcontrol = false;
      coral.manualcontrol = false;
    }
    // DPad Down - Go down a level
    else if (dPad.getDPadDownPressed()) {
      elevator.level -=1;    
      elevator.level = Common.clamp(elevator.level, 0, 5);  
      elevator.manualcontrol = false;
      algae.manualcontrol = false;
      coral.manualcontrol = false;
    }

    // Y & A Buttons - manual control elevator Up and Down
    if (controller.getYButton()) {         
      elevator.raise();} 
    else if (controller.getAButton()) { 
      elevator.lower();} 
    else if (elevator.manualcontrol) {
      elevator.stop();}

    
    // Bumpers - manually control Algae Wrist
    if (controller.getRightBumperButton()) {
        algae.wristraise(); 
        algae.manualcontrol = true;}  
    else if (controller.getLeftBumperButton()) {
        algae.wristlower();
        algae.manualcontrol = true; } 
    else {      
      if (algae.manualcontrol) {
        algae.wriststop(); }}     
        
    // X and B - manually control Coral Wrist
    if (controller.getXButton()) {
        coral.wristlower(); 
        coral.manualcontrol = true;}
    else if (controller.getBButton()) {      
        coral.wristraise(); 
        coral.manualcontrol = true;} 
    else {
      if (coral.manualcontrol) {
        coral.wriststop(); }}        

    // Triggers - control intake and outtake based on Mode
    if (controller.getRightTriggerAxis() > 0.05) {  
      if (CoralMode) {
        coral.outtake(); } 
      else {
        algae.outtake(); }} 
    else if (controller.getLeftTriggerAxis() > 0.05) {
      if (CoralMode) {
        coral.intake(); } 
      else {
        algae.intake(); }} 
    else {
      algae.stop();
      coral.stop(); }
    
    // Press and release Right Stick Button to toggle slow mode
    if (controller.getRightStickButtonPressed()){
      slowmode = !slowmode;
      if (slowmode) {
        slowspeedlimit = 0.5;}
      else {
        slowspeedlimit = 1.0; }
    }

    // Back button - Zero Heading
    if (controller.getBackButtonPressed()) {
      swerveDrive.zeroHeading(); }

      
    // Start button - Toggles field relative
    if (controller.getStartButtonPressed()) {            
      fieldRelative = !fieldRelative; }

    // Get control values from the controller, apply speed limits and deadband
    strafe = MathUtil.applyDeadband(controller.getLeftX() * OIConstants.kDriverSpeedLimit * elevator.elevatorspeedlimiter * slowspeedlimit, OIConstants.kDriveDeadband);
    forward = MathUtil.applyDeadband(-controller.getLeftY() * OIConstants.kDriverSpeedLimit * elevator.elevatorspeedlimiter * slowspeedlimit, OIConstants.kDriveDeadband);
    rotate = MathUtil.applyDeadband(controller.getRightX() * OIConstants.kDriverRotationLimit * slowspeedlimit, OIConstants.kDriveDeadband);


    // Press and hold left stick button for auto alignment
    if (controller.getLeftStickButton()) {
      vision.getDirectionsToTarget();
      forward = vision.forward;
      strafe = vision.strafe;
      rotate = vision.rotate;
    }



    // Send values to swerve drive    
    swerveDrive.drive(forward, strafe, rotate, fieldRelative, rateLimit);

    // Run elevator if not in manual control
    elevator.teleopPeriodic(CoralMode);
    coral.teleopPeriodic(CoralMode, elevator.level);
    algae.teleopPeriodic(!CoralMode, elevator.level);
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

  /* TEST INIT */
  @Override
  public void testInit() { }

  /* TEST PERIODIC */
  @Override
  public void testPeriodic() {}
}