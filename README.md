# 2025_Swerve_vision

This project contains the base swerve drive train implementation plus pathplanner and Limelight vision support. This
represents a superset of the base swerve branch. The implementation can be extracted and included
in a new project that needs a swerve drive train.

To run the autonomous path, open the Shuffleboard UI and load the Competition tab. The tab should
display a selection both with "DO NOTHING!" and "Vision Test" options (you may need to restart shuffleboard for the
widget to display properly). Select the "Vision Test" option. On the Driver Station, select "Autonomous" mode
in the left-hand panel, then select "Enable". The robot should drive in a roughly square pattern (open
the "Vision Test" auto in pathplanner to view), then stop.

## Branches
* main
    * This is the drive train of the TyRapXXIV final Orlando software code updated with addresses and constants
    for the Sparky Junior bot. It is stored here separately to give students a chance to understand the swerve drive code on its own especially for future robot implementations. This code has been verified on the Junior robot. Attempts to use it on another robot will likely need retuning of the swerve drives.
* 2024Swerve_Jake
    * Jake is trying to rebuild the TyRapXXIV swerve drive in his own way.
* 2024Practice
    * Jake is using a test-bed to develop teaching tools for other students.
* 2025_swerve
    * This is the base swerve implemenation for WPILib 2025
* 2025_swerve_pathplanner
    # This is the base swerve implementation plus path planning support

## [Important Info: CANCoders and Tuning](src/main/java/frc/robot/README.md)
