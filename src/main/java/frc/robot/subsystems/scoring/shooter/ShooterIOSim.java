package frc.robot.subsystems.scoring.shooter;

import edu.wpi.first.units.measure.Voltage;

public class ShooterIOSim implements ShooterIO {
  /**
   * Run the Shooter flywheel with a certain voltage applied to the motor
   *
   * @param voltage Voltage to apply to the motor
   */
  @Override
  public void runOpenLoop(Voltage voltage) {
    System.out.println("spinning spinner at " + voltage);
  }

  /** Stop the Shooter flywheel */
  @Override
  public void stop() {
    System.out.println("stopped spinner");
  }
}
