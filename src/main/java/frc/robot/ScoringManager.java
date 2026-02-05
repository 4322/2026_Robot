package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


// Borrowed from 4322 2025 season code
public class ScoringManager {
    private GenericHID rightController;
  private GenericHID leftController;

  public ScoringManager(int rightPort, int leftPort) {
    rightController = new GenericHID(rightPort);
    leftController = new GenericHID(leftPort);
  }

  public GenericHID getRightController() {
    return rightController;
  }

  public GenericHID getLeftController() {
    return leftController;
  }
  public void configScoringPosButtons() {

    /*
    new JoystickButton(rightController, 5)
        .onTrue(
            new InstantCommand(
                    () -> {
                      setScoringLocation(ScoringLocation.A);
                    })
                .ignoringDisable(true));
    */
  }
}
