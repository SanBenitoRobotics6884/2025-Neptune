package frc.robot.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve;

public class AutonomousCommand  extends Command {
    private final Swerve m_swerve;

    public AutonomousCommand(Swerve s_Swerve){
        addRequirements(s_Swerve);
        m_swerve = s_Swerve;
    }

    public void autonomousInit() {
    }

    public void autonomousPeriodic() {
        double targetSpeed = 2.0/*meters*/ / 30/*seconds*/;
        m_Swerve.drive(
            new Translation2d(-1, 0).times(targetSpeed),
            0,
            1,
            true
        );
    }
}
