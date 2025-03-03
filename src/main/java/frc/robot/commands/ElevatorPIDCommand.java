package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPIDCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private double setPoint;

  public ElevatorPIDCommand(ElevatorSubsystem elevatorSubsystem, double setPoint) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.setPoint = setPoint;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    elevatorSubsystem.goToSetpoint(setPoint);
  }

  @Override
  public void end(boolean Interrupted) {
    elevatorSubsystem.stopMotor();
  }

  @Override
  public boolean isFinished() {
    if (elevatorSubsystem.atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }
}
