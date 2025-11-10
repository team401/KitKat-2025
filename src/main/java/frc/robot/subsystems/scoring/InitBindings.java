package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.scoring.shooter.ScoringSubsystem;

/** Methods to initialize bindings for each subsystems */
public final class InitBindings {
  /** This class cannot be instantiated. */
  private InitBindings() {}

  /**
   * Initialize bindings that require only the scoring subsystem
   *
   * <p>This method will assume controller and scoring subsystem are not null.
   */
  public static void initScoringBindings(
      CommandXboxController controller, ScoringSubsystem scoring) {
    controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  scoring.spin();
                  System.out.println("SPINNING!");
                }))
        .onFalse(new InstantCommand(() -> scoring.stop()));
  }
}
