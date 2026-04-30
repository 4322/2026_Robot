package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.rollers.Rollers;

public class TesterRollers extends Command {
  private Rollers rollers;
  private String testName;
  private Color leaderColorStatus;
  private Color followerColorStatus;
  private String leaderStatus;
  private String followerStatus;

  public TesterRollers(Rollers rollers, String testName) {
    this.rollers = rollers;
    this.testName = testName;
  }

  @Override
  public void initialize() {
    leaderColorStatus = Constants.NetworkTables.purple;
    followerColorStatus = Constants.NetworkTables.purple;
    leaderStatus = "";
    followerStatus = "";
    setColorStatus();
    setTextStatus();
    SmartDashboard.putString("Tester/Rollers", testName);
  }

  @Override
  public void execute() {
    if (!rollers.leaderConnected()) {
      leaderColorStatus = Constants.NetworkTables.red;
      leaderStatus = "Not Connected";
    } else if (!rollers.leaderAtGoal()) {
      leaderColorStatus = Constants.NetworkTables.orange;
      leaderStatus =
          "Slow by"
              + String.format(
                  "%.1f",
                  100
                      - Math.abs(rollers.getLeaderSpeed())
                          / Math.abs(rollers.getRequestedSetpoint())
                          * 100)
              + "%";
    } else {
      leaderColorStatus = Constants.NetworkTables.green;
      leaderStatus = "Up To Speed";
    }

    if (!rollers.followerConnected()) {
      followerColorStatus = Constants.NetworkTables.red;
      followerStatus = "Not Connected";
    } else if (!rollers.followerAtGoal()) {
      followerColorStatus = Constants.NetworkTables.orange;
      followerStatus =
          "Slow by"
              + String.format(
                  "%.1f",
                  100
                      - Math.abs(rollers.getFollowerSpeed())
                          / Math.abs(rollers.getRequestedSetpoint())
                          * 100)
              + "%";
      ;
    } else {
      followerColorStatus = Constants.NetworkTables.green;
      followerStatus = "Up To Speed";
    }

    if (!rollers.isCurrentConsistent()) {
      followerColorStatus = Constants.NetworkTables.orange;
      leaderColorStatus = Constants.NetworkTables.orange;
      followerStatus = "Current Inconsistent" + String.format("%.1f", rollers.getFollowerCurrent());
      leaderStatus = "Current Inconsistent" + String.format("%.1f", rollers.getLeaderCurrent());
    }

    setColorStatus();
    setTextStatus();
  }

  private void setColorStatus() {
    SmartDashboard.putString(
        Constants.Tester.rollerColorKeyLeader, leaderColorStatus.toHexString());
    SmartDashboard.putString(
        Constants.Tester.rollerColorKeyFollower, followerColorStatus.toHexString());
  }

  private void setTextStatus() {
    SmartDashboard.putString(Constants.Tester.rollerKeyFollower, followerStatus);
    SmartDashboard.putString(Constants.Tester.rollerKeyLeader, leaderStatus);
    leaderStatus = "";
    followerStatus = "";
  }
}
