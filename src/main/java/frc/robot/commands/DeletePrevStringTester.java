package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DeletePrevStringTester extends Command {
  private String DriveKeyFL = "Tester/Drive/Drive/Front Drive Left Status";
  private String DriveKeyFR = "Tester/Drive/Drive/Front Drive Right Status";
  private String DriveKeyBL = "Tester/Drive/Drive/Back Drive Left Status";
  private String DriveKeyBR = "Tester/Drive/Drive/Back Drive Right Status";

  private String TurnKeyFL = "Tester/Drive/Turn/Front Turn Left Status";
  private String TurnKeyFR = "Tester/Drive/Turn/Front Turn Right Status";
  private String TurnKeyBL = "Tester/Drive/Turn/Back Turn Left Status";
  private String TurnKeyBR = "Tester/Drive/Turn/Back Turn Right Status";
  private String RollerKeyLeader = "Tester/Intake/Rollers/Leader Roller Status";
  private String RollerKeyFollower = "Tester/Intake/Rollers/Follower Roller Status";
  private String currentLeaderFlywheelStatusKey =
      "Tester/Shooter/Flywheel/Leader Flywheel Color Status";
  private String currentFollowerFlywheelStatusKey =
      "Tester/Shooter/Flywheel/Follower Flywheel Color Status";

  public DeletePrevStringTester() {}

  @Override
  public void initialize() {
    SmartDashboard.putString(DriveKeyFL, " ");
    SmartDashboard.putString(DriveKeyFR, " ");
    SmartDashboard.putString(DriveKeyBL, " ");
    SmartDashboard.putString(DriveKeyBR, " ");
    SmartDashboard.putString(TurnKeyFL, " ");
    SmartDashboard.putString(TurnKeyFR, " ");
    SmartDashboard.putString(TurnKeyBL, " ");
    SmartDashboard.putString(TurnKeyBR, " ");
    SmartDashboard.putString(RollerKeyLeader, " ");
    SmartDashboard.putString(RollerKeyFollower, " ");
    SmartDashboard.putString(currentLeaderFlywheelStatusKey, " ");
    SmartDashboard.putString(currentFollowerFlywheelStatusKey, " ");
  }
}
