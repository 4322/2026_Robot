import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;

public class TesterDrive extends Command {
  private Drive drive;
  private String currentFLStatus = "Front Left:";
  private String currentFRStatus = "Front Right:";
  private String currentBLStatus = "Back Left:";
  private String currentBRStatus = "Back Right:";
  private String test;
  private double numberTest;
  private String frontLeftKey = "Tester/Drive/Drive/FL Drive Color Status";
  private String frontRightKey = "Tester/Drive/Drive/FR Drive Color Status";
  private String backLeftKey = "Tester/Drive/Drive/BL Drive Color Status";
  private String backRightKey = "Tester/Drive/Drive/BR Drive Color Status";
  private String driveConnectionMessage = " Drive Not Connected";

  // If Not working = color
  // If working = green
  // If not connected = red
  // If too slow = orange
  // If pulling too much current = blue;
  // still running Tests = purple
  public TesterDrive(Drive drive, String test) {
    this.drive = drive;
    this.test = test;
  }

  @Override
  public void initialize() {

    this.currentFLStatus = this.currentFLStatus + " Running Test " + test;
    this.currentFRStatus = this.currentFRStatus + " Running Test " + test;
    this.currentBLStatus = this.currentBLStatus + " Running Test " + test;
    this.currentBRStatus = this.currentBRStatus + " Running Test " + test;
    this.numberTest = 0;
  }

  @Override
  public void execute() {
    SmartDashboard.putString(frontLeftKey, Constants.NetworkTables.red.kMediumPurple.toHexString());
    SmartDashboard.putString(
        frontRightKey, Constants.NetworkTables.red.kMediumPurple.toHexString());
    SmartDashboard.putString(backLeftKey, Constants.NetworkTables.red.kMediumPurple.toHexString());
    SmartDashboard.putString(backRightKey, Constants.NetworkTables.red.kMediumPurple.toHexString());
    this.currentFLStatus = this.currentFLStatus + " #" + (numberTest + 1);
    this.currentFRStatus = this.currentFRStatus + " #" + (numberTest + 1);
    this.currentBLStatus = this.currentBLStatus + " #" + (numberTest + 1);
    this.currentBRStatus = this.currentBRStatus + " #" + (numberTest + 1);

    if (!drive.isDriveConnected(0)) {
      SmartDashboard.putString(frontLeftKey, Constants.NetworkTables.red.toHexString());
      this.currentFLStatus = this.currentFLStatus + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(0)) {
      SmartDashboard.putString(frontLeftKey, Constants.NetworkTables.red.kOrange.toHexString());
      this.currentFLStatus =
          this.currentFLStatus
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(0) / drive.requestedSpeed) * 100))
              + "%";
    } else {
      SmartDashboard.putString(frontLeftKey, Constants.NetworkTables.green.toHexString());
    }

    if (!drive.isDriveConnected(1)) {
      SmartDashboard.putString(frontRightKey, Constants.NetworkTables.red.toHexString());
      this.currentFRStatus = this.currentFRStatus + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(1)) {
      SmartDashboard.putString(frontRightKey, Constants.NetworkTables.red.kOrange.toHexString());
      this.currentFRStatus =
          this.currentFRStatus
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(1) / drive.requestedSpeed) * 100))
              + "%";
    } else {
      SmartDashboard.putString(frontRightKey, Constants.NetworkTables.green.toHexString());
    }

    if (!drive.isDriveConnected(2)) {
      SmartDashboard.putString(backLeftKey, Constants.NetworkTables.red.kBlack.toHexString());
      this.currentBLStatus = this.currentBLStatus + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(2)) {
      SmartDashboard.putString(backLeftKey, Constants.NetworkTables.red.kOrange.toHexString());
      this.currentBLStatus =
          this.currentBLStatus
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(2) / drive.requestedSpeed) * 100))
              + "%";

    } else {
      SmartDashboard.putString(backLeftKey, Constants.NetworkTables.green.toHexString());
    }

    if (!drive.isDriveConnected(3)) {
      SmartDashboard.putString(backRightKey, Constants.NetworkTables.red.kBlack.toHexString());
      this.currentBRStatus = this.currentBRStatus + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(3)) {
      SmartDashboard.putString(backRightKey, Constants.NetworkTables.red.kOrange.toHexString());
      this.currentBRStatus =
          this.currentBRStatus
              + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(3) / drive.requestedSpeed) * 100))
              + "%";
    } else {
      SmartDashboard.putString(backRightKey, Constants.NetworkTables.green.toHexString());
    }
    SmartDashboard.putString("Tester/Drive/Drive/Front Drive Left Status", this.currentFLStatus);
    SmartDashboard.putString("Tester/Drive/Drive/Front Drive Right Status", this.currentFRStatus);
    SmartDashboard.putString("Tester/Drive/Drive/Back Drive Left Status", this.currentBLStatus);
    SmartDashboard.putString("Tester/Drive/Drive/Back Drive Right Status", this.currentBRStatus);
    this.currentFLStatus = "Front Left:";
    this.currentFRStatus = "Front Right:";
    this.currentBLStatus = "Back Left:";
    this.currentBRStatus = "Back Right:";
  }

  @Override
  public void end(boolean interrupted) {}
}
