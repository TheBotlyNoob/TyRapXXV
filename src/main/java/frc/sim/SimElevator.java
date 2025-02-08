package frc.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.Elevator;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimElevator extends Elevator{

    protected Mechanism2d elevatorModel; // Sim
    protected MechanismLigament2d elevatorPiece; // Sim
    
    protected double speed;
    protected double heightM = ElevatorConstants.minHeightM;
    
    public SimElevator() {
        elevatorModel = new Mechanism2d(.1, ElevatorConstants.minHeightM);
        MechanismRoot2d root = elevatorModel.getRoot("base", .2, .2);
        elevatorPiece = root.append(new MechanismLigament2d("elevator", ElevatorConstants.minHeightM, 90));
        SmartDashboard.putData("ElevatorModel",elevatorModel);
    }

    public void setSpeed(double newSpeed) {
        this.speed = Math.copySign(Math.min(Math.abs(newSpeed), ElevatorConstants.maxVelocityMps), newSpeed);
    }

    public void reset() {
        this.elevatorPiece.setLength(ElevatorConstants.minHeightM);
        this.speed = 0.0;
        this.heightM = ElevatorConstants.minHeightM;
    }

    @Override
    public void simulationPeriodic() {
        this.heightM = MathUtil.clamp(this.heightM + 0.02*this.speed, 
            ElevatorConstants.minHeightM, ElevatorConstants.maxHeightM);
        this.elevatorPiece.setLength(this.heightM);
    }
}

