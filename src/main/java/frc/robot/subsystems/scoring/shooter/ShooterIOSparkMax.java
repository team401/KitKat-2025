package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.scoring.shooter.ShooterConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.scoring.shooter.ShooterIO.ShooterInputs;

public class ShooterIOSparkMax implements ShooterIO {
  private SparkMax motor;

  public ShooterIOSparkMax() {

    final int motorID = 6;

    // Configure motor
    motor = new SparkMax(motorID, MotorType.kBrushless);

    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);

    // Apply config to leaders
    config.inverted(true);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void stop() {
    motor.set(0);
  }

  public void updateInputs(ShooterInputs inputs) {
    ifOk(
        motor,
        () -> motor.getEncoder().getVelocity(),
        (double velocity) -> inputs.motorVelocity.mut_replace(velocity, RPM));

    // TODO: Use ifOk for the rest of these
    inputs.motorAppliedVolts.mut_replace(motor.getAppliedOutput(), Volts);
    inputs.motorCurrent.mut_replace(motor.getOutputCurrent(), Amps);
  }

  public void runOpenLoop(Voltage voltage) {
    motor.setVoltage(voltage);
  }
}
