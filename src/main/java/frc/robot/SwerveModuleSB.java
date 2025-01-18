package frc.robot;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

class SwerveModuleSB
{
    private GenericEntry mPositionEntry;
    private GenericEntry mAngleEntry;
    private GenericEntry mDesiredAngleEntry;
    private GenericEntry mSpeedDelta;
    private SwerveModule mModule;
    
    
    public SwerveModuleSB(String name, SwerveModule module, ShuffleboardTab tab) {
        mPositionEntry = tab.add(name + " Position", 0).getEntry();
        mAngleEntry = tab.add(name + " Angle", 0).getEntry();
        mDesiredAngleEntry = tab.add(name + "D Angle", 0).getEntry();
        mSpeedDelta = tab.add(name + " Spd Dta", 0).getEntry();
        mModule = module;
    }

    public void update()
    {
        mAngleEntry.setDouble(mModule.getPosition().angle.getDegrees());
        mPositionEntry.setDouble(mModule.getPosition().distanceMeters);
        mDesiredAngleEntry.setDouble(mModule.getDesiredState().angle.getDegrees());
        mSpeedDelta.setDouble(mModule.getState().speedMetersPerSecond - mModule.getDesiredState().speedMetersPerSecond);
    }
}