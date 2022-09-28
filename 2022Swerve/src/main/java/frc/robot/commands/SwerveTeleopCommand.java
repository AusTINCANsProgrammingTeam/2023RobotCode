package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTeleopCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xJoystick, yJoystick, rJoystick;

    public SwerveTeleopCommand(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xJoystick, Supplier<Double> yJoystick, Supplier<Double> rJoystick) {
        this.swerveSubsystem = swerveSubsystem;
        this.xJoystick = xJoystick;
        this.yJoystick = yJoystick;
        this.rJoystick = rJoystick;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        SwerveModuleState[] moduleStates = 
            swerveSubsystem.convertToModuleStates(xJoystick.get(), yJoystick.get(), rJoystick.get());

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
