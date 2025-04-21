package frc.robot.Subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRetrievalIO {
  @AutoLog
  public static class AlgaeRetrievalIOInputs {}

  public default void updateInputs(AlgaeRetrievalIOInputs inputs) {}

  public default void setSpeed(double speed) {}

  public default void setVoltage(double voltage) {}
}
