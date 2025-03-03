package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private static SparkMax sparkMax2 = new SparkMax(ClimberConstants.FrontClimbCanID, MotorType.kBrushless);
    private static SparkMax sparkMax3 = new SparkMax(ClimberConstants.BackClimbCanID, MotorType.kBrushless);

    public ClimberSubsystem() {
        
    }

    public Command climberCommand(double speed) {
        return run(() -> {
            sparkMax2.set(speed);
            sparkMax3.set(speed);
        });
    }
}
