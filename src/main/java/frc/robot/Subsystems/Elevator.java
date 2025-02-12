package frc.robot.Subsystems;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final SparkMax leadMotor;
    private final SparkMax followMotor;
    private ElevatorFeedforward elevatorFF;
    private ProfiledPIDController elevatorPID;
    private double pidOutput;
    private double ffOutput;

    public Elevator(){
        this.elevatorPID = new ProfiledPIDController(
        ElevatorConstants.kp, 
        ElevatorConstants.ki, 
        ElevatorConstants.kd, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));

        this.elevatorFF = new ElevatorFeedforward(
        ElevatorConstants.ks,
        ElevatorConstants.kg,
        ElevatorConstants.kv,
        ElevatorConstants.ka);
        //set up motors
        leadMotor = new SparkMax(ElevatorConstants.leadCANID, MotorType.kBrushless);
        leadMotor.getEncoder().setPosition(0);
        SparkMaxConfig leadConfig = new SparkMaxConfig();
        leadConfig.smartCurrentLimit(40);
        leadConfig.idleMode(IdleMode.kBrake); 
        leadConfig.inverted(false);
        leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        followMotor = new SparkMax(ElevatorConstants.followCANID, MotorType.kBrushless);
        followMotor.getEncoder().setPosition(0);
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.smartCurrentLimit(40); 
        followConfig.idleMode(IdleMode.kBrake);
        followMotor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        followConfig.follow(ElevatorConstants.leadCANID, false);
    }

    public double getActualHeight(){
        return this.leadMotor.getEncoder().getPosition() / ElevatorConstants.kElevatorGearing * 
        (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius); //double check this
    }

    public void goToHeight(int goalHeight){
        TrapezoidProfile.State setpoint = elevatorPID.getSetpoint();
        //m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal); //0.020 ms
        pidOutput = this.elevatorPID.calculate(this.getActualHeight(), goalHeight); //this might be wrong
        ffOutput = this.elevatorFF.calculate(setpoint.velocity);
        leadMotor.setVoltage(MathUtil.clamp(pidOutput + ffOutput, -ElevatorConstants.kMaxVoltage, ElevatorConstants.kMaxVoltage));
        //clamp might not be needed
    }

    @Override
    public void periodic() {

    }
}
