package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;

public class DPadHelper {
    private final XboxController controller;
    private int lastPOVDOWN = -1;
    private int lastPOVUP = -1;
    private int lastPOVLEFT = -1;
    private int lastPOVRIGHT = -1;
    private int current; 
    private boolean result;   

    private NetworkTableEntry NTDPadUpPressed;

    public DPadHelper(XboxController controller) {
        this.controller = controller;
        NetworkTable Table = NetworkTableInstance.getDefault().getTable("DPadTest");
        NTDPadUpPressed = Table.getEntry("DPadUp");
    }

    public boolean getDPadUpPressed() {
        current = controller.getPOV();        
        result = ((current == 0) && (lastPOVUP != 0));
        NTDPadUpPressed.setInteger(current);
        lastPOVUP = current;
        return result;
    }

    public boolean getDPadDownPressed() {
        current = controller.getPOV();
        result = (current == 180 && lastPOVDOWN != 180);
        lastPOVDOWN = current;
        return result;
    }

    public boolean getDPadLeftPressed() {
      current = controller.getPOV();
      result = (current == 270 && lastPOVLEFT != 270);
      lastPOVLEFT = current;
      return result;
    }
  
    public boolean getDPadRightPressed() {
      current = controller.getPOV();
      result = (current == 90 && lastPOVRIGHT != 90);
      lastPOVRIGHT = current;
      return result;
    }
}
