package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterInputs {
    public MutAngularVelocity motorVelocity = RotationsPerSecond.mutable(0.0);
    public MutVoltage motorAppliedVolts = Volts.mutable(0.0);
    public MutCurrent motorStatorCurrent = Amps.mutable(0.0);
    public MutCurrent motorSupplyCurrent = Amps.mutable(0.0);
  }

  /**
   * Refresh and read all status signals from motors, updating a ShooterInputs object with new
   * values
   *
   * @param inputs The ShooterInputs object to update
   */
  public default void updateInputs(ShooterInputs inputs) {}

  /**
   * Run the Shooter flywheel with a certain voltage applied to the motor
   *
   * @param voltage Voltage to apply to the motor
   */
  public default void runOpenLoop(Voltage voltage) {}

  /** Stop the Shooter flywheel */
  public default void stop() {}
}
