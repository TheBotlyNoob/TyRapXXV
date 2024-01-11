package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.CloseCone;
import frc.robot.Commands.CloseCube;
import frc.robot.Commands.CloseLift;
import frc.robot.Commands.OpenCone;
import frc.robot.Commands.OpenCube;
import frc.robot.Commands.OpenLift;

public class Pneumatics extends SubsystemBase {
    private DoubleSolenoid m_lift;
    private DoubleSolenoid m_cube;
    private DoubleSolenoid m_cone;
    private boolean m_coneClose = false;
    private boolean m_cubeClose = false;
    private boolean m_liftOut = false;

    public Pneumatics() {
        this.m_lift = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3);
        this.m_cube = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 4, 5);
        this.m_cone = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
    }

    public Command openConeCommand() {
        System.out.println("OPEN CONE!\n");
        return new OpenCone(this);
    }
    public Command closeConeCommand() {
        System.out.println("CLOSE CONE!\n");
        return new CloseCone(this);
    }
    public Command openCubeCommand() {
        System.out.println("OPEN CUBE!\n");
        return new OpenCube(this);
    }
    public Command closeCubeCommand() {
        System.out.println("CLOSE CUBE!\n");
        return new CloseCube(this);
    }
    public Command openLiftCommand() {
        return new OpenLift(this);
    }
    public Command closeLiftCommand() {
        return new CloseLift(this);
    }

    public void enableLift() {
        this.m_lift.set(Value.kForward);
        this.liftOut(false);
    }

    public void disableLift() {
        this.m_lift.set(Value.kReverse);
        this.liftOut(true);
    }

    public void enableCone() {
        this.m_cone.set(Value.kForward);
        this.coneClosed(false);
    }

    public void disableCone() {
        this.m_cone.set(Value.kReverse);
        this.coneClosed(true);
    }

    public void enableCube() {
        this.m_cube.set(Value.kForward);
        this.cubeClosed(false);
    }

    public void disableCube() {
        this.m_cube.set(Value.kReverse);
        this.cubeClosed(true);
    }

    public boolean coneClosed() {
        return this.m_coneClose;
    }

    public void coneClosed(boolean isConeClosed) {
        this.m_coneClose = isConeClosed;
    }

    public boolean cubeClosed() {
        return m_cubeClose;
    }

    public void cubeClosed(boolean isCubeClosed) {
        this.m_cubeClose = isCubeClosed;
    }

    public boolean liftOut() {
        return m_liftOut;
    }

    public void liftOut(boolean isLiftOut) {
        this.m_liftOut = isLiftOut;
    }
}
