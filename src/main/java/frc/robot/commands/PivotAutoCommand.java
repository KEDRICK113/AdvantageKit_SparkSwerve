package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class PivotAutoCommand extends Command{
    
    private final PivotSubsystem m_PivotSubsystem;
    private final double time;
    private final Timer timer = new Timer();
    private final boolean direction;
    private boolean Finished;
    
    public PivotAutoCommand(PivotSubsystem pivotSubsystem, double timee, boolean direction) {
        m_PivotSubsystem = pivotSubsystem;
        time = timee;
        addRequirements(m_PivotSubsystem);
        this.direction=direction;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        Finished = false;
    }

    @Override
    public void execute() {
        if (timer.get() < time) {
            double speed = direction ? PivotConstants.PivotSpeed : -1 * PivotConstants.PivotSpeed;
            m_PivotSubsystem.setSpeed(speed);
        } else {
            Finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_PivotSubsystem.setSpeed(0);    
    }

    @Override
    public boolean isFinished() {
        return Finished;
    }
}
