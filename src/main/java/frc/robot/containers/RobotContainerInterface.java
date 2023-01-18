package frc.robot.containers;

import edu.wpi.first.wpilibj2.command.Command;

public interface RobotContainerInterface {
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand();
}

