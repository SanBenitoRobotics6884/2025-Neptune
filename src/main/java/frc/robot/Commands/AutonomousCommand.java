package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousCommand  extends SequentialCommandGroup {
    private final Swerve m_swerve;
    SmartDashboard m_smartdashboard;
    public AutonomousCommand(Swerve s_Swerve){
        addRequirements(s_Swerve);
        m_swerve = s_Swerve;

        addCommands(
            m_swerve.driveForwardCommand()
        );
    }
}
