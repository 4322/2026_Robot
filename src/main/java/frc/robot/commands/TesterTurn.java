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
  private Color FLColorStatus = new Color(0, 0, 0);
  private Color FRColorStatus = new Color(0, 0, 0);
  private Color BLColorStatus = new Color(0, 0, 0);
  private Color BRColorStatus = new Color(0, 0, 0);
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
    this.FLColorStatus = Constants.NetworkTables.purple;
    this.FRColorStatus = Constants.NetworkTables.purple;
    this.BLColorStatus = Constants.NetworkTables.purple;
    this.BRColorStatus = Constants.NetworkTables.purple;
  }

  @Override
  public void execute() {

    if (!drive.isTurnConnected(0)) {
      this.FLStatus = turnConnectionMessage;
      this.FLColorStatus = Constants.NetworkTables.red;
    } else if (!drive.isCorrectAngleSpeed(0)) {
      this.FLColorStatus = Constants.NetworkTables.orange;
      this.FLStatus =
          " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleTurnVelocity(0) / drive.anglePerSecondRequested) * 100))
              + "%";
    } else {
      this.FLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isTurnConnected(1)) {
      this.FRColorStatus = Constants.NetworkTables.red;
      this.FRStatus = turnConnectionMessage;
    } else if (!drive.isCorrectAngleSpeed(1)) {
      this.FRColorStatus = Constants.NetworkTables.orange;
      this.FRStatus =
          this.FRStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleTurnVelocity(1) / drive.anglePerSecondRequested) * 100))
              + "%";
    } else {
      this.FRColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isTurnConnected(2)) {
      this.BLColorStatus = Constants.NetworkTables.red;
      this.BLStatus = turnConnectionMessage;
    } else if (!drive.isCorrectAngleSpeed(2)) {
      this.BLColorStatus = Constants.NetworkTables.orange;
      this.BLStatus =
          this.BLStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleTurnVelocity(2) / drive.anglePerSecondRequested) * 100))
              + "%";

    } else {
      this.BLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isTurnConnected(3)) {
      this.BRColorStatus = Constants.NetworkTables.red;
      this.BRStatus = turnConnectionMessage;
    } else if (!drive.isCorrectAngleSpeed(3)) {
      this.BRColorStatus = Constants.NetworkTables.orange;
      this.BRStatus =
          this.BRStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleTurnVelocity(3) / drive.anglePerSecondRequested) * 100))
              + "%";
    } else {
      this.BRColorStatus = Constants.NetworkTables.green;
    }

    SmartDashboard.putString(
        Constants.Tester.TurnKeyFL,
        SmartDashboard.getString(Constants.Tester.TurnKeyFL, "") + test + this.FLStatus);
    SmartDashboard.putString(
        Constants.Tester.TurnKeyFR,
        SmartDashboard.getString(Constants.Tester.TurnKeyFR, "") + test + this.FRStatus);
    SmartDashboard.putString(
        Constants.Tester.TurnKeyBL,
        SmartDashboard.getString(Constants.Tester.TurnKeyBL, "") + test + this.BLStatus);
    SmartDashboard.putString(
        Constants.Tester.TurnKeyBR,
        SmartDashboard.getString(Constants.Tester.TurnKeyBR, "") + test + this.BRStatus);

    SmartDashboard.putString(Constants.Tester.TurnColorKeyFL, this.FLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.TurnColorKeyFR, this.FRColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.TurnColorKeyBL, this.BLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.TurnColorKeyBR, this.BRColorStatus.toHexString());
  }

  @Override
  public void end(boolean interrupted) {}
}
