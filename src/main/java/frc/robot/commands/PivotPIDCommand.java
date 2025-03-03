package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotPIDCommand extends Command {
  private final PivotSubsystem pivotSubsystem;
  private double setPoint;

  public PivotPIDCommand(PivotSubsystem pivotSubsystem, double setPoint) {
    this.pivotSubsystem = pivotSubsystem;
    this.setPoint = setPoint;

    addRequirements(pivotSubsystem);
  }

  @Override
  public void execute() {
    pivotSubsystem.goToSetpoint(setPoint);
  }

  @Override
  public void end(boolean Interrupted) {
    pivotSubsystem.stopMotor();
  }

  @Override
  public boolean isFinished() {
    if (pivotSubsystem.atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }
}
