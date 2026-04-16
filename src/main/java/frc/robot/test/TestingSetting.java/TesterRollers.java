import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.rollers.Rollers;

public class TesterRollers extends Command {
  private Rollers rollers;
  private String test;
  private String currentLeaderRollerStatusKey = "Tester/Intake/Rollers/Leader Roller Color Status";
  private String currentFollowerRollerStatusKey =
      "Tester/Intake/Rollers/Follower Roller Color Status";
  private String rollerLeader = " Roller Leader: ";
  private String rollerFollower = " Roller Follower: ";
  private double rollerTest;

  public TesterRollers(Rollers rollers, String test) {
    this.rollers = rollers;
    this.test = test;
  }

  @Override
  public void initialize() {
    SmartDashboard.putString(
        currentLeaderRollerStatusKey, Constants.NetworkTables.purple.toHexString());
    SmartDashboard.putString(
        currentFollowerRollerStatusKey, Constants.NetworkTables.purple.toHexString());
    rollerLeader = rollerLeader + " Running Test " + test;
  }

  @Override
  public void execute() {
    rollerLeader = rollerLeader + " Running Test " + (rollerTest + 1);
    rollerFollower = rollerFollower + " Running Test " + (rollerTest + 1);
    rollers.setRollerSpeed(0.5);

    if (!rollers.leaderRollerConnected()) {
      SmartDashboard.putString(
          currentLeaderRollerStatusKey, Constants.NetworkTables.red.toHexString());
      rollerLeader = rollerLeader + " Not Connected";
    } else if (rollers.leaderRollerAtGoal()) {
      SmartDashboard.putString(
          currentLeaderRollerStatusKey, Constants.NetworkTables.orange.toHexString());
      rollerLeader =
          rollerLeader
              + "Not Spinning at Goal by"
              + +(100 - ((rollers.getLeaderRollerSpeed() / rollers.getRequestedSetpoint()) * 100))
              + "%";
      ;
    }

    if (!rollers.followerRollerConnected()) {
      SmartDashboard.putString(
          currentFollowerRollerStatusKey, Constants.NetworkTables.red.toHexString());
      rollerFollower = rollerFollower + " Not Connected";
    } else if (!rollers.followerRollerConnected()) {
      SmartDashboard.putString(
          currentFollowerRollerStatusKey, Constants.NetworkTables.orange.toHexString());
      rollerFollower =
          rollerFollower
              + "Not Spinning at Goal by"
              + +(100 - ((rollers.getFollowerRollerSpeed() / rollers.getRequestedSetpoint()) * 100))
              + "%";
      ;
    } else {
      SmartDashboard.putString(
          currentFollowerRollerStatusKey, Constants.NetworkTables.orange.toHexString());
      rollerFollower = rollerFollower + " Not Connected";
    }

    if (!rollers.rollersSpinningTogether()) {
      rollerLeader = rollerLeader + " Rollers Not Spinning Together";
      rollerFollower = rollerFollower + " Rollers Not Spinning Together";
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
              + Math.abs(rollers.getLeaderRollerSpeed() - rollers.getFollowerRollerSpeed())
              + " RPS";
      rollerFollower =
          rollerFollower
              + "Diffrence of "
              + Math.abs(rollers.getLeaderRollerSpeed() - rollers.getFollowerRollerSpeed())
              + " RPS";
    } else {
      SmartDashboard.putString(
          currentLeaderRollerStatusKey, Constants.NetworkTables.green.toHexString());
      SmartDashboard.putString(
          currentFollowerRollerStatusKey, Constants.NetworkTables.green.toHexString());
      rollerLeader = rollerLeader + " Rollers Spinning Together";
      rollerFollower = rollerFollower + " Rollers Spinning Together";
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
