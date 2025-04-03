package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Common;



public class VisionSubsystem extends SubsystemBase{

    // PhotonVision 
    PhotonCamera camera;
    boolean targetVisible;  
    int tagid;
    int bestTarget;
    double largestArea;
    double targetForwardDistance;
    double targetStrafeDistance;
    double targetYaw;
    double targetPitch;
    boolean aligned;

    public double forward;
    public double strafe;
    public double rotate;

    public VisionSubsystem() {
        camera = new PhotonCamera("FrontLeftCamera"); }

    public void init() {
        camera.getLatestResult(); // warm-up  
        forward=0;
        strafe=0;
        rotate=0;    }

    public void getDirectionsToTarget() {
      forward = 0;
      strafe = 0;
      rotate = 0;
      getResults();

      if (targetVisible) {
        forward = (8.0 - largestArea) / largestArea; 
        strafe= -(-5.94-targetYaw)*.02; 
        forward = Common.clamp(forward, -0.1, 0.1, 0.05); 
        strafe = Common.clamp(strafe, -0.05, 0.05, 0.01);        
        rotate = strafe;
        aligned = (forward == 0) && (rotate == 0);
      } 
    }

    public void getResults() {
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
                    bestTarget = tagid;
                    largestArea = target.getArea();
                    targetPitch = target.getPitch();
                    targetYaw = target.getYaw();
                    targetVisible = true;}}}}}
    }

    public boolean targetVisible() {
        return targetVisible;
    }

    public boolean onTarget() {
        return targetVisible && aligned;
    }
}
