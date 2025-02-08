package frc.sim;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimCoralManipulator extends SubsystemBase {
    protected MechanismLigament2d fourBarElement;
    
    public SimCoralManipulator(SimElevator elevator) {

        fourBarElement = new MechanismLigament2d("fourBar", 0.4, -45.0);
        fourBarElement.setColor(new Color8Bit(0, 0, 255));
        elevator.elevatorPiece.append(fourBarElement);
    }
    
}
