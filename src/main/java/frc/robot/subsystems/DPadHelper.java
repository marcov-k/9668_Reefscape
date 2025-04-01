package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;

public class DPadHelper {
    private final XboxController controller;
    private int lastPOV = -1;
    private int current; 
    private boolean result;   

    public DPadHelper(XboxController controller) {
        this.controller = controller;
    }

    public void update() {
        lastPOV = controller.getPOV();
    }

    public boolean getDPadUpPressed() {
        current = controller.getPOV();
        result = (current == 0 && lastPOV != 0);
        lastPOV = current;
        return result;
    }

    public boolean getDPadDownPressed() {
        current = controller.getPOV();
        result = (current == 180 && lastPOV != 180);
        lastPOV = current;
        return result;
    }

    public boolean getDPadLeftPressed() {
      current = controller.getPOV();
      result = (current == 270 && lastPOV != 270);
      lastPOV = current;
      return result;
    }
  
    public boolean getDPadRightPressed() {
      current = controller.getPOV();
      result = (current == 90 && lastPOV != 90);
      lastPOV = current;
      return result;
    }
}
