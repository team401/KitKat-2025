package frc.robot.subsystems.scoring.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.subsystems.scoring.shooter.ShooterConstants.*;
import static frc.robot.util.SparkUtil.*;

public class ShooterIOSparkMax {
    private SparkMax motor;
    
    public ShooterIOSparkMax() {

        final int motorID = 5;

        // Configure motor
        motor = new SparkMax(motorID, MotorType.kBrushless); 
        
        var config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
        config.closedLoop.pidf(realKp, 0.0, realKd, 0.0);
        
        // Apply config to leaders
        //config.inverted(leftInverted); ?????????????????
        tryUntilOk(
            motor,
            5,
            () ->
                motor.configure(
                    config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }
    
    public void startUpMotor(double speed) {
        motor.set(speed);
    }
    
    public void stopMotor() {
        motor.set(0);
    }

}
