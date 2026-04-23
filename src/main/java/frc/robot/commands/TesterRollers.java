package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.rollers.Rollers;
import org.littletonrobotics.junction.Logger;

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
    Logger.recordOutput("Tester/Rollers", testName);
  }

  @Override
  public void execute() {
    followerStatus = " ";
    leaderStatus = " ";
    if (!rollers.leaderRollerConnected()) {
      leaderColorStatus = Constants.NetworkTables.red;
      leaderStatus = " Not Connected";
    } else if (!rollers.leaderRollerAtGoal()) {
      leaderColorStatus = Constants.NetworkTables.orange;
      leaderStatus =
          "Not Spinning at Goal by"
              + +(100 - ((rollers.getLeaderRollerSpeed() / rollers.getRequestedSetpoint()) * 100))
              + "%";
    } else {
      leaderColorStatus = Constants.NetworkTables.green;
      leaderStatus = "Up To Speed";
    }

    if (!rollers.followerRollerConnected()) {
      followerColorStatus = Constants.NetworkTables.red;
      followerStatus = " Not Connected";
    } else if (!rollers.followerRollerAtGoal()) {
      followerColorStatus = Constants.NetworkTables.orange;
      followerStatus =
          "Not Spinning at Goal by"
              + +(100 - ((rollers.getFollowerRollerSpeed() / rollers.getRequestedSetpoint()) * 100))
              + "%";
      ;
    } else {
      followerColorStatus = Constants.NetworkTables.green;
      followerStatus = "Up To Speed";
    }

    if (!rollers.rollersSpinningTogether()) {
      leaderStatus = " Rollers Not Spinning Together";
      followerStatus = " Rollers Not Spinning Together";
      leaderStatus =
          leaderStatus
              + " Leader at "
              + rollers.getLeaderRollerSpeed()
              + " RPS, Follower at "
              + rollers.getFollowerRollerSpeed()
              + " RPS";
      followerStatus =
          followerStatus
              + " Leader at "
              + rollers.getLeaderRollerSpeed()
              + " RPS, Follower at "
              + rollers.getFollowerRollerSpeed()
              + " RPS";
      leaderStatus =
          leaderStatus
              + "Diffrence of "
              + (rollers.getLeaderRollerSpeed() - rollers.getFollowerRollerSpeed())
              + " RPS";
      followerStatus =
          followerStatus
              + "Diffrence of "
              + (rollers.getFollowerRollerSpeed() - rollers.getLeaderRollerSpeed())
              + " RPS";
    } else {
      followerColorStatus = Constants.NetworkTables.green;
      leaderColorStatus = Constants.NetworkTables.green;
      leaderStatus = leaderStatus + " Rollers Spinning Together";
      followerStatus = followerStatus + " Rollers Spinning Together";
    }

    setColorStatus();
    setTextStatus();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  private void setColorStatus() {
    SmartDashboard.putString(
        Constants.Tester.RollerColorKeyLeader, leaderColorStatus.toHexString());
    SmartDashboard.putString(
        Constants.Tester.RollerColorKeyFollower, followerColorStatus.toHexString());
  }

  private void setTextStatus() {
    SmartDashboard.putString(Constants.Tester.RollerKeyFollower, followerStatus);
    SmartDashboard.putString(Constants.Tester.RollerKeyLeader, leaderStatus);
  }
}
