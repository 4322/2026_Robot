package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;

public class TesterTurn extends Command {
  private Drive drive;
  private String FLStatus = "";
  private String FRStatus = "";
  private String BLStatus = "";
  private String BRStatus = "";
  private Color FLColorStatus;
  private Color FRColorStatus;
  private Color BLColorStatus;
  private Color BRColorStatus;
  private String test;
  private String turnConnectionMessage = " Turn Not Connected";

  // If Not working = color
  // If working = green
  // If not connected = red
  // If too slow = orange
  // If pulling too much current = blue;
  // still running Tests = purple
  public TesterTurn(Drive drive, String test) {
    this.drive = drive;
    this.test = test;
  }

  @Override
  public void initialize() {
    SmartDashboard.putString(
        Constants.Tester.TurnKeyFL, Constants.NetworkTables.purple.toHexString());
    SmartDashboard.putString(
        Constants.Tester.TurnKeyFR, Constants.NetworkTables.purple.toHexString());
    SmartDashboard.putString(
        Constants.Tester.TurnKeyBL, Constants.NetworkTables.purple.toHexString());
    SmartDashboard.putString(
        Constants.Tester.TurnKeyBR, Constants.NetworkTables.purple.toHexString());
    FLColorStatus = Constants.NetworkTables.purple;
    FRColorStatus = Constants.NetworkTables.purple;
    BLColorStatus = Constants.NetworkTables.purple;
    BRColorStatus = Constants.NetworkTables.purple;
  }

  @Override
  public void execute() {

    if (!drive.isTurnConnected(0)) {
      FLStatus = turnConnectionMessage;
      FLColorStatus = Constants.NetworkTables.red;
    } else if (!drive.isCorrectAngleSpeed(0)) {
      FLColorStatus = Constants.NetworkTables.orange;
      FLStatus =
          " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleTurnVelocity(0) / drive.anglePerSecondRequested) * 100))
              + "%";
    } else {
      FLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isTurnConnected(1)) {
      FRColorStatus = Constants.NetworkTables.red;
      FRStatus = turnConnectionMessage;
    } else if (!drive.isCorrectAngleSpeed(1)) {
      FRColorStatus = Constants.NetworkTables.orange;
      FRStatus =
          FRStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleTurnVelocity(1) / drive.anglePerSecondRequested) * 100))
              + "%";
    } else {
      FRColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isTurnConnected(2)) {
      BLColorStatus = Constants.NetworkTables.red;
      BLStatus = turnConnectionMessage;
    } else if (!drive.isCorrectAngleSpeed(2)) {
      BLColorStatus = Constants.NetworkTables.orange;
      BLStatus =
          BLStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleTurnVelocity(2) / drive.anglePerSecondRequested) * 100))
              + "%";

    } else {
      BLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isTurnConnected(3)) {
      BRColorStatus = Constants.NetworkTables.red;
      BRStatus = turnConnectionMessage;
    } else if (!drive.isCorrectAngleSpeed(3)) {
      BRColorStatus = Constants.NetworkTables.orange;
      BRStatus =
          BRStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleTurnVelocity(3) / drive.anglePerSecondRequested) * 100))
              + "%";
    } else {
      BRColorStatus = Constants.NetworkTables.green;
    }

    SmartDashboard.putString(
        Constants.Tester.TurnKeyFL,
        SmartDashboard.getString(Constants.Tester.TurnKeyFL, "") + test + FLStatus);
    SmartDashboard.putString(
        Constants.Tester.TurnKeyFR,
        SmartDashboard.getString(Constants.Tester.TurnKeyFR, "") + test + FRStatus);
    SmartDashboard.putString(
        Constants.Tester.TurnKeyBL,
        SmartDashboard.getString(Constants.Tester.TurnKeyBL, "") + test + BLStatus);
    SmartDashboard.putString(
        Constants.Tester.TurnKeyBR,
        SmartDashboard.getString(Constants.Tester.TurnKeyBR, "") + test + BRStatus);

    SmartDashboard.putString(Constants.Tester.TurnColorKeyFL, FLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.TurnColorKeyFR, FRColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.TurnColorKeyBL, BLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.TurnColorKeyBR, BRColorStatus.toHexString());
  }

  @Override
  public void end(boolean interrupted) {}
}
