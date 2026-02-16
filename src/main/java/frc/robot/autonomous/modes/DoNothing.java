package frc.robot.autonomous.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.hood.Hood;

public class DoNothing extends SequentialCommandGroup {
  public DoNothing(Hood hood) {
    setName("DO_NOTHING");
    addCommands(new InstantCommand(() -> hood.requestGoal(0.0)));
  }
}
