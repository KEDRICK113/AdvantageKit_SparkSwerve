package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    
    private static SparkMax sparkMax13 = new SparkMax(PivotConstants.PivotCanID, MotorType.kBrushless);
    private static AbsoluteEncoder encoder;
    private static SparkMaxConfig pivotConfig = new SparkMaxConfig();
    private static PIDController PivotPID = new PIDController(.01, 0, 0);

    public PivotSubsystem() {
        pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(MotorConstants.AmpLimitNeo).inverted(true);
        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        pivotConfig.absoluteEncoder.positionConversionFactor(360).inverted(true);
        sparkMax13.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = sparkMax13.getAbsoluteEncoder();
        PivotPID.setTolerance(1);
        PivotPID.setIZone(3);
    }

    public void goToSetpoint(double setpoint) {
        double speed = PivotPID.calculate(getEncoderPosition(), setpoint);
        PIDSetSpeed(speed);
        SmartDashboard.putNumber("Pivot Speed", speed);
    }

    public boolean atSetpoint() {
        return PivotPID.atSetpoint();
    }
    
    public AbsoluteEncoder getAbsoluteEncoder() {
        return encoder;
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public void stopMotor() {
        sparkMax13.set(0);
    }

    public void PIDSetSpeed(double speed) {
        if (speed > PivotConstants.PivotMaxSpeed) {
            speed = PivotConstants.PivotMaxSpeed;
        } if (speed < -PivotConstants.PivotMaxSpeed) {
            speed = -PivotConstants.PivotMaxSpeed;
        }
        sparkMax13.set(speed);
    }

    public void setSpeed(double speed) {
        sparkMax13.set(speed);
    }

    public Command pivotCommand(double speed) {
        return run(() -> {
            sparkMax13.set(speed);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Angle", getEncoderPosition());
    }
}
