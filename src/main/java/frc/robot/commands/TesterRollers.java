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
  private Color leaderRollerColorStatus;
  private Color followerRollerColorStatus;
  private String rollerLeader = "  ";
  private String rollerFollower = "  ";

  public TesterRollers(Rollers rollers, String testName) {
    this.rollers = rollers;
    this.testName = testName;
  }

  @Override
  public void initialize() {
    leaderRollerColorStatus = Constants.NetworkTables.purple;
    followerRollerColorStatus = Constants.NetworkTables.purple;
    SmartDashboard.putString(
        Constants.Tester.RollerKeyLeader, Constants.NetworkTables.purple.toHexString());
    SmartDashboard.putString(
        Constants.Tester.RollerKeyFollower, Constants.NetworkTables.purple.toHexString());
    Logger.recordOutput("Tester/Rollers", testName);
  }

  @Override
  public void execute() {
    rollerFollower = " ";
    rollerLeader = " ";
    if (!rollers.leaderRollerConnected()) {
      leaderRollerColorStatus = Constants.NetworkTables.red;
      rollerLeader = " Not Connected";
    } else if (!rollers.leaderRollerAtGoal()) {
      leaderRollerColorStatus = Constants.NetworkTables.orange;
      rollerLeader =
          "Not Spinning at Goal by"
              + +(100 - ((rollers.getLeaderRollerSpeed() / rollers.getRequestedSetpoint()) * 100))
              + "%";
    } else {
      leaderRollerColorStatus = Constants.NetworkTables.green;
    }

    if (!rollers.followerRollerConnected()) {
      followerRollerColorStatus = Constants.NetworkTables.red;
      rollerFollower = " Not Connected";
    } else if (!rollers.followerRollerAtGoal()) {
      followerRollerColorStatus = Constants.NetworkTables.orange;
      rollerFollower =
          "Not Spinning at Goal by"
              + +(100 - ((rollers.getFollowerRollerSpeed() / rollers.getRequestedSetpoint()) * 100))
              + "%";
      ;
    } else {
      followerRollerColorStatus = Constants.NetworkTables.green;
    }

    if (!rollers.rollersSpinningTogether()) {
      rollerLeader = " Rollers Not Spinning Together";
      rollerFollower = " Rollers Not Spinning Together";
      rollerLeader =
          rollerLeader
              + " Leader at "
              + rollers.getLeaderRollerSpeed()
              + " RPS, Follower at "
              + rollers.getFollowerRollerSpeed()
              + " RPS";
      rollerFollower =
          rollerFollower
              + " Leader at "
              + rollers.getLeaderRollerSpeed()
              + " RPS, Follower at "
              + rollers.getFollowerRollerSpeed()
              + " RPS";
      rollerLeader =
          rollerLeader
              + "Diffrence of "
              + (rollers.getLeaderRollerSpeed() - rollers.getFollowerRollerSpeed())
              + " RPS";
      rollerFollower =
          rollerFollower
              + "Diffrence of "
              + (rollers.getFollowerRollerSpeed() - rollers.getLeaderRollerSpeed())
              + " RPS";
    } else {
      followerRollerColorStatus = Constants.NetworkTables.green;
      leaderRollerColorStatus = Constants.NetworkTables.green;
      rollerLeader = rollerLeader + " Rollers Spinning Together";
      rollerFollower = rollerFollower + " Rollers Spinning Together";
    }

    SmartDashboard.putString(
        Constants.Tester.RollerColorKeyLeader, leaderRollerColorStatus.toHexString());
    SmartDashboard.putString(
        Constants.Tester.RollerColorKeyFollower, followerRollerColorStatus.toHexString());

    SmartDashboard.putString(
        Constants.Tester.RollerKeyFollower,
        SmartDashboard.getString(Constants.Tester.RollerKeyFollower, "")
            + testName
            + rollerFollower);
    SmartDashboard.putString(
        Constants.Tester.RollerKeyLeader,
        SmartDashboard.getString(Constants.Tester.RollerKeyLeader, "") + testName + rollerLeader);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
