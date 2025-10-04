package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

public class ShooterConstants {
  public static final int currentLimit = 60;
  // Velocity PID configuration
  public static final double realKp = 0.0;
  public static final double realKd = 0.0;
  public static final double realKs = 0.0;
  public static final double realKv = 0.1;

  public static final double simKp = 0.05;
  public static final double simKd = 0.0;
  public static final double simKs = 0.0;
  public static final double simKv = 0.227;
  public static final Time deviceConnectedDebounceTime = Seconds.of(0.5);

  public static final TalonFXConfiguration baseTalonFXConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(Amps.of(40.0))
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(40.0))
                  .withStatorCurrentLimitEnable(true))
          .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true))
          .withSlot0(
              new Slot0Configs()
                  .withKP(100.0) // TODO: Tune gains in real life
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(0.0)
                  .withKG(0.0)
                  .withKV(0.01)
                  .withKA(10.0))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(80)));

  public static final Frequency updateFrequency = Hertz.of(50.0);
  public static final Integer maxConfigApplyAttempts = 5;
  public static final Double configApplyTimeoutSeconds = 0.25;
}
