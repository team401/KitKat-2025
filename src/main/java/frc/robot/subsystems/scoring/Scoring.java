package frc.robot.subsystems.scoring;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Scoring extends SubsystemBase {
  SparkFlex rollerSparkFlex = new SparkFlex(6, MotorType.kBrushless);

  public enum ScoringAction {
    Idle,
    Score
  }

  private ScoringAction currentAction = ScoringAction.Idle;

  @Override
  public void periodic() {
    switch (currentAction) {
      case Idle -> {
        rollerSparkFlex.setVoltage(0.0);
      }
      case Score -> {
        rollerSparkFlex.setVoltage(3.0);
      }
    }
  }

  public void setAction(ScoringAction action) {
    this.currentAction = action;
  }
}
