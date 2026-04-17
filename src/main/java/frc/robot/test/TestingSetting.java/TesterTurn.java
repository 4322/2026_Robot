import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;

public class TesterTurn extends Command {
  private Drive drive;
  private String currentFLStatus = "Front Left:";
  private String currentFRStatus = "Front Right:";
  private String currentBLStatus = "Back Left:";
  private String currentBRStatus = "Back Right:";
  private Color currentFLColorStatus = new Color(0, 0, 0);
  private Color currentFRColorStatus = new Color(0, 0, 0);
  private Color currentBLColorStatus = new Color(0, 0, 0);
  private Color currentBRColorStatus = new Color(0, 0, 0);
  private String test;
  private double numberTest;
  private String frontLeftKey = "Tester/Drive/Turn/FL Turn Color Status";
  private String frontRightKey = "Tester/Drive/Turn/FR Turn Color Status";
  private String backLeftKey = "Tester/Drive/Turn/BL Turn Color Status";
  private String backRightKey = "Tester/Drive/Turn/BR Turn Color Status";
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
    SmartDashboard.putString(frontLeftKey, Constants.NetworkTables.purple.toHexString());
    SmartDashboard.putString(frontRightKey, Constants.NetworkTables.purple.toHexString());
    SmartDashboard.putString(backLeftKey, Constants.NetworkTables.purple.toHexString());
    SmartDashboard.putString(backRightKey, Constants.NetworkTables.purple.toHexString());
    this.currentFLColorStatus = Constants.NetworkTables.purple;
    this.currentFRColorStatus = Constants.NetworkTables.purple;
    this.currentBLColorStatus = Constants.NetworkTables.purple;
    this.currentBRColorStatus = Constants.NetworkTables.purple;
  }

  @Override
  public void execute() {
    this.currentFLStatus = " ";
    this.currentFRStatus = " ";
    this.currentBLStatus = " ";
    this.currentBRStatus = " ";

    if (!drive.isTurnConnected(0)) {
      this.currentFLStatus = this.currentFLStatus + turnConnectionMessage;
      this.currentFLColorStatus = Constants.NetworkTables.red;
    } else if (!drive.isCorrectAngleSpeed(0)) {
      this.currentFLColorStatus = Constants.NetworkTables.orange;
      this.currentFLStatus =
          this.currentFLStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleAngle(0) / drive.anglePerSecondRequested) * 100))
              + "%";
    } else {
      this.currentFLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isTurnConnected(1)) {
      this.currentFRColorStatus = Constants.NetworkTables.red;
      this.currentFRStatus = this.currentFRStatus + turnConnectionMessage;
    } else if (!drive.isCorrectAngleSpeed(1)) {
      this.currentFRColorStatus = Constants.NetworkTables.orange;
      this.currentFRStatus =
          this.currentFRStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleAngle(1) / drive.anglePerSecondRequested) * 100))
              + "%";
    } else {
      this.currentFRColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isTurnConnected(2)) {
      this.currentBLColorStatus = Constants.NetworkTables.red;
      this.currentBLStatus = this.currentBLStatus + turnConnectionMessage;
    } else if (!drive.isCorrectAngleSpeed(2)) {
      this.currentBLColorStatus = Constants.NetworkTables.orange;
      this.currentBLStatus =
          this.currentBLStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleAngle(2) / drive.anglePerSecondRequested) * 100))
              + "%";

    } else {
      this.currentBLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isTurnConnected(3)) {
      this.currentBRColorStatus = Constants.NetworkTables.red;
      this.currentBRStatus = this.currentBRStatus + turnConnectionMessage;
    } else if (!drive.isCorrectAngleSpeed(3)) {
      this.currentBRColorStatus = Constants.NetworkTables.orange;
      this.currentBRStatus =
          this.currentBRStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleAngle(3) / drive.anglePerSecondRequested) * 100))
              + "%";
    } else {
      this.currentBRColorStatus = Constants.NetworkTables.green;
    }


    SmartDashboard.putString("Tester/Drive/Turn/Front Turn Left Status", this.currentFLStatus);
    SmartDashboard.putString("Tester/Drive/Turn/Front Turn Right Status", this.currentFRStatus);
    SmartDashboard.putString("Tester/Drive/Turn/Back Turn Left Status", this.currentBLStatus);
    SmartDashboard.putString("Tester/Drive/Turn/Back Turn Right Status", this.currentBRStatus);
    
    SmartDashboard.putString(frontLeftKey, this.currentFLColorStatus.toHexString());
    SmartDashboard.putString(frontRightKey, this.currentFRColorStatus.toHexString());
    SmartDashboard.putString(backLeftKey, this.currentBLColorStatus.toHexString());
    SmartDashboard.putString(backRightKey, this.currentBRColorStatus.toHexString());
  }

  @Override
  public void end(boolean interrupted) {}
}
