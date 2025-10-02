package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ScoringSubsystem extends SubsystemBase {
  private ShooterIO shooterIO;
  private ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

  public ScoringSubsystem(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("scoring/inputs", inputs);
  }

  public void stop() {
    shooterIO.stop();
  }

  public void spin() {
    shooterIO.runOpenLoop(Volts.of(12.0));
  }
}
