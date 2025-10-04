package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.scoring.shooter.ShooterIO.ShooterInputs;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  protected TalonFX motor;

  private Debouncer connectedDebouncer =
      new Debouncer(ShooterConstants.deviceConnectedDebounceTime.in(Seconds));

  // Store TalonFX configs to avoid creating a new one when updating PID/FF gains in tuning mode
  private TalonFXConfiguration talonFXConfigs;

  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<AngularAcceleration> motorAcceleration;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Double> motorClosedLoopOutput;
  private final StatusSignal<Double> motorClosedLoopReference;
  private final StatusSignal<Current> motorSupplyCurrent;
  private final StatusSignal<Current> motorStatorCurrent;

  // Store control requests to avoid creating new ones every cycle
  private final TorqueCurrentFOC focRequest = new TorqueCurrentFOC(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final MotionMagicVelocityTorqueCurrentFOC closedLoopRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  // Store an alert to publish if configs fail to apply
  private final Alert configFailedToApplyAlert;

  public ShooterIOTalonFX() {
    final int motorID = 6;

    // Configure motors
    motor = new TalonFX(motorID, "canivore");

    talonFXConfigs = ShooterConstants.baseTalonFXConfigs;

    InvertedValue motorInvert = InvertedValue.Clockwise_Positive;

    talonFXConfigs.MotorOutput.Inverted = motorInvert;

    applyMotorConfig();

    // Create status signals
    motorVelocity = motor.getRotorVelocity();
    motorAcceleration = motor.getAcceleration();
    motorVoltage = motor.getMotorVoltage();
    motorClosedLoopOutput = motor.getClosedLoopOutput();
    motorClosedLoopReference = motor.getClosedLoopReference();
    motorSupplyCurrent = motor.getSupplyCurrent();
    motorStatorCurrent = motor.getStatorCurrent();

    // Configure status signal updates
    BaseStatusSignal.setUpdateFrequencyForAll(
        ShooterConstants.updateFrequency,
        motorVelocity,
        motorAcceleration,
        motorVoltage,
        motorClosedLoopOutput,
        motorClosedLoopReference,
        motorSupplyCurrent,
        motorStatorCurrent);

    // Only update the signals configured above, and reduce all frequencies to the configured values
    ParentDevice.optimizeBusUtilizationForAll(motor);

    // Initialize alert(s) with proper name
    configFailedToApplyAlert = new Alert(" shooter IO failed to apply configs.", AlertType.kError);
    configFailedToApplyAlert.set(false);
  }

  /**
   * Apply the current talonFXConfigs to the motor, trying to re-apply until it succeeds
   *
   * <p>If the config fails to apply after all attempts, an alert will be shown and an error will be
   * printed.
   */
  private void applyMotorConfig() {
    PhoenixUtil.tryUntilOk(
        ShooterConstants.maxConfigApplyAttempts,
        () ->
            motor
                .getConfigurator()
                .apply(talonFXConfigs, ShooterConstants.configApplyTimeoutSeconds));
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    StatusCode status =
        BaseStatusSignal.refreshAll(
            motorVelocity,
            motorAcceleration,
            motorVoltage,
            motorClosedLoopOutput,
            motorClosedLoopReference,
            motorSupplyCurrent,
            motorStatorCurrent);

    // Update inputs
    inputs.motorVelocity.mut_replace(motorVelocity.getValue());
    inputs.motorAppliedVolts.mut_replace(motorVoltage.getValue());
    inputs.motorSupplyCurrent.mut_replace(motorSupplyCurrent.getValue());
    inputs.motorStatorCurrent.mut_replace(motorStatorCurrent.getValue());
  }

  @Override
  public void runOpenLoop(Voltage voltage) {
    motor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
