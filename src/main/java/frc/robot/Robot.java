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
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DPadHelper;
import frc.utils.Common;



public class Robot extends TimedRobot {

  // Time  
  long elapsedTime;
  long startTime;

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
  boolean aligned;

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
    camera = new PhotonCamera("FrontLeftCamera");
    camera.getLatestResult(); // warm-up   
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
  }

  /* AUTONOMOUS PERIODIC */
  @Override
  public void autonomousPeriodic() {
    // Initialize Variables 
    elapsedTime = System.currentTimeMillis() - startTime;
    forward = 0.0;
    strafe = 0.0;
    rotate = 0.0; 
    targetForwardDistance = 0.0;
    targetStrafeDistance = 0.0;
    targetYaw = 0.0;
    targetVisible = false;      
    
    
    // Drive forward for two seconds
    if (elapsedTime < 2000) {
      forward = 0.05; }
    
    // For the next 5 seconds rotate until you see a Reef AprilTag, then rotate and drive towards it
    else if (elapsedTime < 15000 && !aligned) {      
      largestArea = 0.0;
      var results = camera.getAllUnreadResults();

      if (!results.isEmpty()){
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            for (var target : result.getTargets()) {
                tagid = target.getFiducialId();
                // If the tag is a reef tag
                if ((tagid > 6 && tagid < 11) || (tagid > 17 && tagid < 21)) {
                  // Find the closest reef tag (based on largest area of frame) and capture Yaw and Range
                  if (target.getArea() > largestArea) {
                    largestArea = target.getArea();
                    bestTarget = tagid;
                    targetTransform = target.getBestCameraToTarget();
                    targetTranslation = targetTransform.getTranslation();
                    targetRotation = targetTransform.getRotation();
                    targetForwardDistance = targetTranslation.getZ();
                    targetStrafeDistance = targetTranslation.getX();
                    targetYaw = Math.toDegrees(targetRotation.getZ());
                    targetVisible = true;}}}}}

      if (targetVisible) {
        forward = Common.clamp(targetForwardDistance - 0.3, -0.3, 0.3, 0.05);        
        strafe = Common.clamp(targetStrafeDistance, -0.3, 0.3, 0.05);
        rotate = Common.clamp(targetYaw / 30.0, -0.1, 0.1, 0.02); }

      aligned = Math.abs(targetForwardDistance - 0.3) < 0.05 && Math.abs(targetStrafeDistance) < 0.05 && Math.abs(targetYaw) < 3.0; }
    
    else if (aligned) {
      elevator.goToCoralLevel(2);
      coral.scoringpose(); 
      coral.outtake();
    }
    
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
  }

  /* TELEOP PERIODIC */
  @Override
  public void teleopPeriodic() {
    
    // DPad Left - Select Coral Mode 
    if (dPad.getDPadLeftPressed()) {
      CoralMode = true;      
      coral.unfold(elevator.level);
      algae.fold();
      elevator.manualcontrol = false;
      algae.manualcontrol = false;
      coral.manualcontrol = false;
      }
    // DPad Right - Select Algae Mode 
    else if (dPad.getDPadRightPressed()) {
      CoralMode = false; 
      coral.fold();
      algae.unfold();
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

    // Y Button - manual control elevator Up and Down
    if (controller.getYButton()) {         
      elevator.raise();} 
    else if (controller.getAButton()) { 
      elevator.lower();} 
    else if (elevator.manualcontrol) {
      elevator.stop();}

    
    // Bumpers - manually control Algae or Coral Wrist
    if (controller.getRightBumperButton()) {
      if (CoralMode){
        coral.wristraise(); } 
      else {
        algae.wristraise(); } } 
    else if (controller.getLeftBumperButton()) {
      if (CoralMode){        
        coral.wristlower(); } 
      else {
        algae.wristlower(); }} 
    else {
      if (coral.manualcontrol) {
        coral.wriststop(); }      
      if (algae.manualcontrol) {
        algae.wriststop(); }}      

    // Triggers - control intake and outtake
    if (controller.getRightTriggerAxis() > 0.05) {  
      if (CoralMode) {
        coral.intake(); } 
      else {
        algae.intake(); }} 
    else if (controller.getLeftTriggerAxis() > 0.05) {
      if (CoralMode) {
        coral.outtake(); } 
      else {
        algae.outtake(); }} 
    else {
      algae.stop();
      coral.stop(); }
    
    // Back button - Zero Heading
    if (controller.getBackButtonPressed()) {
      swerveDrive.zeroHeading(); }

      
    // Start button - Toggles field relative
    if (controller.getStartButtonPressed()) {            
      fieldRelative = !fieldRelative; }

    // Get control values from the controller, apply speed limits and deadband
    strafe = MathUtil.applyDeadband(controller.getLeftX() * OIConstants.kDriverSpeedLimit * elevator.elevatorspeedlimiter, OIConstants.kDriveDeadband);
    forward = MathUtil.applyDeadband(-controller.getLeftY() * OIConstants.kDriverSpeedLimit * elevator.elevatorspeedlimiter, OIConstants.kDriveDeadband);
    rotate = MathUtil.applyDeadband(controller.getRightX() * OIConstants.kDriverRotationLimit, OIConstants.kDriveDeadband);


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