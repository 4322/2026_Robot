package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.rollers.Rollers;

public class TesterRollers extends Command {
  private Rollers rollers;
  private String test;
  private Color currentLeaderRollerColorStatus = Constants.NetworkTables.purple;
  private Color currentFollowerRollerColorStatus = Constants.NetworkTables.purple;
  private String rollerLeader = " Roller Leader: ";
  private String rollerFollower = " Roller Follower: ";

  public TesterRollers(Rollers rollers, String test) {
    this.rollers = rollers;
    this.test = test;
  }

  @Override
  public void initialize() {
    currentLeaderRollerColorStatus = Constants.NetworkTables.purple;
    currentFollowerRollerColorStatus = Constants.NetworkTables.purple;
    SmartDashboard.putString(
        Constants.Tester.RollerKeyLeader, Constants.NetworkTables.purple.toHexString());
    SmartDashboard.putString(
        Constants.Tester.RollerKeyFollower, Constants.NetworkTables.purple.toHexString());
  }

  @Override
  public void execute() {
    rollerFollower = " ";
    rollerLeader = " ";
    if (!rollers.leaderRollerConnected()) {
      currentLeaderRollerColorStatus = Constants.NetworkTables.red;
      rollerLeader = " Not Connected";
    } else if (!rollers.leaderRollerAtGoal()) {
      currentLeaderRollerColorStatus = Constants.NetworkTables.orange;
      rollerLeader =
          "Not Spinning at Goal by"
              + +(100 - ((rollers.getLeaderRollerSpeed() / rollers.getRequestedSetpoint()) * 100))
              + "%";
    } else {
      currentLeaderRollerColorStatus = Constants.NetworkTables.green;
    }

    if (!rollers.followerRollerConnected()) {
      currentFollowerRollerColorStatus = Constants.NetworkTables.red;
      rollerFollower = " Not Connected";
    } else if (!rollers.followerRollerAtGoal()) {
      currentFollowerRollerColorStatus = Constants.NetworkTables.orange;
      rollerFollower =
          "Not Spinning at Goal by"
              + +(100 - ((rollers.getFollowerRollerSpeed() / rollers.getRequestedSetpoint()) * 100))
              + "%";
      ;
    } else {
      currentFollowerRollerColorStatus = Constants.NetworkTables.green;
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
      currentFollowerRollerColorStatus = Constants.NetworkTables.green;
      currentLeaderRollerColorStatus = Constants.NetworkTables.green;
      rollerLeader = rollerLeader + " Rollers Spinning Together";
      rollerFollower = rollerFollower + " Rollers Spinning Together";
    }

    SmartDashboard.putString(
        Constants.Tester.RollerColorKeyLeader, currentLeaderRollerColorStatus.toHexString());
    SmartDashboard.putString(
        Constants.Tester.RollerColorKeyFollower, currentFollowerRollerColorStatus.toHexString());

    SmartDashboard.putString(
        Constants.Tester.RollerKeyFollower,
        SmartDashboard.getString(Constants.Tester.RollerKeyFollower, "") + test + rollerFollower);
    SmartDashboard.putString(
        Constants.Tester.RollerKeyLeader,
        SmartDashboard.getString(Constants.Tester.RollerKeyLeader, "") + test + rollerLeader);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
