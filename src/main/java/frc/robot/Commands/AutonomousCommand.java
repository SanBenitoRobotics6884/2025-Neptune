package frc.robot.Commands;
import frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousCommand extends Command {
    private final Swerve m_swerve;
    SmartDashboard m_smartdashboard;
    public AutonomousCommand(Swerve s_Swerve){
        addRequirements(s_Swerve);
        m_swerve = s_Swerve;
    }

    public void autonomousInit() {
    }

    public void autonomousPeriodic() {
        m_smartdashboard.putData(m_swerve);
        double targetSpeed = 2.0/*meters*/ / 30/*seconds*/;
        this.m_swerve.drive(
            new Translation2d(-1, 0).times(targetSpeed),
            0,
            true,
            true
        );

    }
}
