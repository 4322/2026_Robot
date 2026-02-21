package frc.robot.subsystems;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutonomousSelector.AutoName;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Simulator extends SubsystemBase {
  private static final RegressTests regressTest = RegressTests.DOUBLE_DOUBLE;
  public static AutoName autoScenario;
  private TeleopScenario teleopScenario;
  private List<TeleAnomaly> teleAnomalies;
  private List<AutoAnomaly> autoAnomalies;
  private String currentScenario;
  private String gameSpecificMessage = "";
  private final Map<Integer, Double> axisValues = new HashMap<Integer, Double>();
  private Timer warmupTimer = new Timer();
  public static boolean slipWheels = false;

  private enum RegressTests {
    DOUBLE_DOUBLE,
    DOUBLE_AUTO,
    DOUBLE_TELEOP,
    APRIL_TAG_TEST,
    CONTROLLER_TEST
  }

  private enum TeleAnomaly {
    NONE,
    DROP_CORAL1_EARLY,
    DROP_CORAL1_LATE,
    DROP_CORAL2_LATE,
    DROP_ALGAE1_EARLY
  }

  private enum AutoAnomaly {
    NONE
  }

  private enum TeleopScenario {
    NONE,
    SCORE_L4,
    LOOK_FROM_APRILTAG_RED_SIDE,
    CONTROLLER_TEST1,
    CONTROLLER_TEST2
  }

  private enum EventType {
    SET_POSE,
    FLYWHEEL_DETECT_FUEL,
    END_EFFECTOR_DETECT_ALGAE,
    END_EFFECTOR_NO_CORAL,
    END_EFFECTOR_NO_ALGAE,
    CORAL_IN_INDEXER,
    CORAL_NOT_IN_INDEXER,
    CORAL_VISIBLE,
    CORAL_NOT_VISIBLE,
    PRESS_A,
    HOLD_A,
    RELEASE_A,
    PRESS_B,
    HOLD_B,
    RELEASE_B,
    PRESS_X,
    HOLD_X,
    RELEASE_X,
    PRESS_Y,
    HOLD_Y,
    RELEASE_Y,
    PRESS_LEFT_BUMPER,
    HOLD_LEFT_BUMPER,
    RELEASE_LEFT_BUMPER,
    PRESS_RIGHT_BUMPER,
    HOLD_RIGHT_BUMPER,
    RELEASE_RIGHT_BUMPER,
    RELEASE_POV,
    PRESS_UP_POV,
    HOLD_UP_POV,
    PRESS_RIGHT_POV,
    HOLD_RIGHT_POV,
    PRESS_DOWN_POV,
    HOLD_DOWN_POV,
    PRESS_LEFT_POV,
    HOLD_LEFT_POV,
    HOLD_LEFT_TRIGGER,
    RELEASE_LEFT_TRIGGER,
    HOLD_RIGHT_TRIGGER,
    RELEASE_RIGHT_TRIGGER,
    PRESS_LEFT_STICK,
    HOLD_LEFT_STICK,
    RELEASE_LEFT_STICK,
    PRESS_RIGHT_STICK,
    HOLD_RIGHT_STICK,
    RELEASE_RIGHT_STICK,
    MOVE_JOYSTICK_DRIVE,
    MOVE_JOYSTICK_TURN,
    STOP_JOYSTICK,
    ENABLE_WHEEL_SLIP,
    DISABLE_WHEEL_SLIP,
    BLUE_INACTIVE_FIRST,
    RED_INACTIVE_FIRST,
    END_OF_SCENARIO
  }

  private enum EventStatus {
    ACTIVE,
    INACTIVE
  }

  private enum POVDirection {
    NONE(-1),
    UP(0),
    RIGHT(90),
    DOWN(180),
    LEFT(270);

    public final int value;

    private POVDirection(int value) {
      this.value = value;
    }
  }

  private enum ControllerAxis {
    LEFT_X(0),
    LEFT_Y(1),
    LEFT_TRIGGER(2),
    RIGHT_TRIGGER(3),
    RIGHT_X(4),
    RIGHT_Y(5);

    public final int value;

    private ControllerAxis(int value) {
      this.value = value;
    }
  }

  private static class RegressionTest {
    private String name;
    private AutoName autoScenario;
    private List<AutoAnomaly> autoAnomalies;
    private TeleopScenario teleopScenario;
    private List<TeleAnomaly> teleAnomalies;
    private Alliance alliance;

    RegressionTest(
        String name,
        AutoName autoScenario,
        List<AutoAnomaly> autoAnomalies,
        TeleopScenario teleopScenario,
        List<TeleAnomaly> teleAnomalies,
        Alliance alliance) {
      this.name = name;
      this.autoScenario = autoScenario;
      this.autoAnomalies = autoAnomalies;
      this.teleopScenario = teleopScenario;
      this.teleAnomalies = teleAnomalies;
      this.alliance = alliance;
    }

    RegressionTest(
        String name, AutoName autoScenario, List<AutoAnomaly> autoAnomalies, Alliance alliance) {
      this(name, autoScenario, autoAnomalies, null, null, alliance);
    }

    RegressionTest(
        String name,
        AutoName autoScenario,
        TeleopScenario teleopScenario,
        List<TeleAnomaly> teleAnomalies,
        Alliance alliance) {
      this(name, autoScenario, (List<AutoAnomaly>) null, teleopScenario, teleAnomalies, alliance);
    }

    RegressionTest(
        String name, AutoName autoScenario, TeleopScenario teleopScenario, Alliance alliance) {
      this(name, autoScenario, teleopScenario, null, alliance);
    }

    RegressionTest(String name, AutoName autoScenario, Alliance alliance) {
      this(name, autoScenario, null, null, alliance);
    }

    RegressionTest(
        String name,
        TeleopScenario teleopScenario,
        List<TeleAnomaly> teleAnomalies,
        Alliance alliance) {
      this(name, null, teleopScenario, teleAnomalies, alliance);
    }

    RegressionTest(String name, TeleopScenario teleopScenario, Alliance alliance) {
      this(name, teleopScenario, null, alliance);
    }
  }

  // TODO
  private List<RegressionTest> regressionTestCases() {
    return List.of(new RegressionTest("Default", AutoName.DO_NOTHING, Alliance.Blue));
  }

  private class SimEvent {
    private double eventTime;
    private EventStatus eventStatus;
    private String eventName;
    private EventType eventType;
    private Pose2d pose;

    SimEvent(double eventTime, String eventName, EventType eventType) {
      this(eventTime, eventName, eventType, EventStatus.ACTIVE);
    }

    SimEvent(double eventTime, String eventName, EventType eventType, EventStatus eventStatus) {
      this(eventTime, eventName, eventType, (Translation2d) null, eventStatus);
    }

    SimEvent(double eventTime, String eventName, EventType eventType, Translation2d position) {
      this(eventTime, eventName, eventType, new Pose2d(position, Rotation2d.kZero));
    }

    SimEvent(
        double eventTime,
        String eventName,
        EventType eventType,
        Translation2d position,
        EventStatus eventStatus) {
      this(eventTime, eventName, eventType, new Pose2d(position, Rotation2d.kZero), eventStatus);
    }

    SimEvent(double eventTime, String eventName, EventType eventType, Pose2d pose) {
      this(eventTime, eventName, eventType, pose, EventStatus.ACTIVE);
    }

    SimEvent(
        double eventTime,
        String eventName,
        EventType eventType,
        Pose2d pose,
        EventStatus eventStatus) {
      this.eventTime = eventTime;
      this.eventName = eventName;
      this.eventType = eventType;
      this.eventStatus = eventStatus;
      this.pose = pose;
    }
  }

  private List<SimEvent> buildAutoScenario() {
    if (autoScenario == null) {
      return List.of();
    }
    double t = 0.0;
    // TODO
    return switch (autoScenario) {
      default -> List.of();
    };
  }

  private List<SimEvent> buildTeleopScenario() {
    if (teleopScenario == null) {
      return List.of();
    }
    double t = 0.0;
    int eventNum = 1;


    return switch (teleopScenario) {

      case LOOK_FROM_APRILTAG_RED_SIDE -> List.of();

      case CONTROLLER_TEST1 -> List.of(
          new SimEvent(
              t += 1.0, "Start pose", EventType.SET_POSE, new Pose2d(12, 4, Rotation2d.k180deg)),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_X),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_LEFT_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_RIGHT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_X),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_X),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-0.3, 0.3, Rotation2d.k180deg)),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(0, 0.3, Rotation2d.k180deg)),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_RIGHT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.STOP_JOYSTICK),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_UP_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_BUMPER),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-0.3, 0.3, Rotation2d.kZero)),
          // Turn X isn't used, setting it here to test controller data reset
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(1, -0.3, Rotation2d.kZero)),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_X),
          new SimEvent(t += 1.0, "Final Movement", EventType.END_OF_SCENARIO));

      case CONTROLLER_TEST2 -> List.of(
          // check that controls were released from CONTROLLER_TEST1 when switching to this scenario
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_X),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_RIGHT_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_RIGHT_BUMPER),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.4, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(0, -0.3, Rotation2d.kZero)),
          new SimEvent(t += 1.0, "Final Movement", EventType.END_OF_SCENARIO));

      default -> List.of();
    };
  }

  private boolean aListContains(AutoAnomaly aAnomaly) {
    if (autoAnomalies == null) {
      return false;
    }
    return autoAnomalies.contains(aAnomaly);
  }

  private boolean tListContains(TeleAnomaly tAnomaly) {
    if (teleAnomalies == null) {
      return false;
    }
    return teleAnomalies.contains(tAnomaly);
  }

  private Iterator<RegressionTest> regressionTestIterator = regressionTestCases().iterator();
  private RegressionTest currentRegressionTest;
  private List<SimEvent> autoEvents;
  private List<SimEvent> teleopEvents;
  private List<SimEvent> events;
  private Iterator<SimEvent> eventIterator;
  private SimEvent currentEvent;
  private Alliance currentAlliance;
  private final Timer disabledTimer = new Timer();
  private final Timer matchTimer = new Timer();
  XboxController hid = RobotContainer.controller.getHID(); // the real WPILib XboxController
  int hidPort = hid.getPort();
  int activeButtonBitmask;
  int momentaryButtonBitmask;
  int activePOV;
  boolean momentaryPOV;

  private final Drive drive;

  public Simulator(Drive drive) {
    this.drive = drive;

    warmupTimer.start();
  }

  @Override
  public void periodic() {
    // wait for PathPanner and other libraries to initialize
    if (warmupTimer.isRunning()) {
      if (warmupTimer.hasElapsed(4)) {
        warmupTimer.stop();
        configureControllerTestBindings();
        setNextRegressTest();
      } else {
        // keep feeding the simulated controller
        configureController();
        DriverStationSim.notifyNewData();
        return;
      }
    }
    if (disabledTimer.isRunning()) {
      if (!disabledTimer.hasElapsed(2)) {
        // keep feeding the simulated controller
        configureController();
        DriverStationSim.notifyNewData();
        return;
      }
      disabledTimer.stop();
      disabledTimer.reset();
      DriverStationSim.setEnabled(true);
      matchTimer.restart();
      currentEvent = eventIterator.next();
    }

    Logger.recordOutput("Sim/MatchTime", matchTimer.get());
    Logger.recordOutput("Sim/RegressionTest", currentRegressionTest.name);
    Logger.recordOutput("Sim/Alliance", currentAlliance.toString());
    Logger.recordOutput("Sim/Scenario", currentScenario);
    Logger.recordOutput("Sim/MatchTime", matchTimer.get());
    configureController();

    // execute all pending events
    while (currentEvent != null && matchTimer.get() >= currentEvent.eventTime) {
      if (currentEvent.eventStatus == EventStatus.ACTIVE) {
        Logger.recordOutput("Sim/EventName", currentEvent.eventName);
        Logger.recordOutput("Sim/EventType", currentEvent.eventType);
        /*   What we used to have:
        case END_EFFECTOR_NO_CORAL -> endEffectorIOSim.simCoralReleased();
        case END_EFFECTOR_NO_ALGAE -> endEffectorIOSim.simAlgaeReleased();
        case END_EFFECTOR_DETECT_CORAL -> endEffectorIOSim.simCoralHeld();
        case END_EFFECTOR_DETECT_ALGAE -> endEffectorIOSim.simAlgaeHeld();
        case CORAL_IN_PICKUP_AREA -> indexerIOSim.simCoralDetectedInPickupArea();
        case CORAL_NOT_IN_PICKUP_AREA -> indexerIOSim.simCoralNOTDetectedInPickupArea();
        case CORAL_IN_INDEXER -> indexerIOSim.simCoralDetectedInIndexer();
        case CORAL_NOT_IN_INDEXER -> indexerIOSim.simCoralNOTDetectedInIndexer();
        case CORAL_VISIBLE -> visionObjectDetectionIOSim.coralDetected(
        currentEvent.pose.getTranslation(), WPIUtilJNI.now() * 1.0e-6);
        case CORAL_NOT_VISIBLE -> visionObjectDetectionIOSim.noCoral(); */
        switch (currentEvent.eventType) {
          case SET_POSE -> drive.setPose(currentEvent.pose);
          case PRESS_A -> pressButton(XboxController.Button.kA);
          case HOLD_A -> holdButton(XboxController.Button.kA);
          case RELEASE_A -> releaseButton(XboxController.Button.kA);
          case PRESS_B -> pressButton(XboxController.Button.kB);
          case HOLD_B -> holdButton(XboxController.Button.kB);
          case RELEASE_B -> releaseButton(XboxController.Button.kB);
          case PRESS_X -> pressButton(XboxController.Button.kX);
          case HOLD_X -> holdButton(XboxController.Button.kX);
          case RELEASE_X -> releaseButton(XboxController.Button.kX);
          case PRESS_Y -> pressButton(XboxController.Button.kY);
          case HOLD_Y -> holdButton(XboxController.Button.kY);
          case RELEASE_Y -> releaseButton(XboxController.Button.kY);
          case PRESS_LEFT_BUMPER -> pressButton(XboxController.Button.kLeftBumper);
          case HOLD_LEFT_BUMPER -> holdButton(XboxController.Button.kLeftBumper);
          case RELEASE_LEFT_BUMPER -> releaseButton(XboxController.Button.kLeftBumper);
          case PRESS_RIGHT_BUMPER -> pressButton(XboxController.Button.kRightBumper);
          case HOLD_RIGHT_BUMPER -> holdButton(XboxController.Button.kRightBumper);
          case RELEASE_RIGHT_BUMPER -> releaseButton(XboxController.Button.kRightBumper);
          case RELEASE_POV -> holdPOV(POVDirection.NONE);
          case PRESS_UP_POV -> pressPOV(POVDirection.UP);
          case HOLD_UP_POV -> holdPOV(POVDirection.UP);
          case PRESS_RIGHT_POV -> pressPOV(POVDirection.RIGHT);
          case HOLD_RIGHT_POV -> holdPOV(POVDirection.RIGHT);
          case PRESS_DOWN_POV -> pressPOV(POVDirection.DOWN);
          case HOLD_DOWN_POV -> holdPOV(POVDirection.DOWN);
          case PRESS_LEFT_POV -> pressPOV(POVDirection.LEFT);
          case HOLD_LEFT_POV -> holdPOV(POVDirection.LEFT);
          case HOLD_LEFT_TRIGGER -> holdTrigger(ControllerAxis.LEFT_TRIGGER);
          case RELEASE_LEFT_TRIGGER -> releaseTrigger(ControllerAxis.LEFT_TRIGGER);
          case HOLD_RIGHT_TRIGGER -> holdTrigger(ControllerAxis.RIGHT_TRIGGER);
          case RELEASE_RIGHT_TRIGGER -> releaseTrigger(ControllerAxis.RIGHT_TRIGGER);
          case PRESS_LEFT_STICK -> pressButton(XboxController.Button.kLeftStick);
          case HOLD_LEFT_STICK -> holdButton(XboxController.Button.kLeftStick);
          case RELEASE_LEFT_STICK -> releaseButton(XboxController.Button.kLeftStick);
          case PRESS_RIGHT_STICK -> pressButton(XboxController.Button.kRightStick);
          case HOLD_RIGHT_STICK -> holdButton(XboxController.Button.kRightStick);
          case RELEASE_RIGHT_STICK -> releaseButton(XboxController.Button.kRightStick);
          case MOVE_JOYSTICK_DRIVE -> {
            persistAxis(ControllerAxis.LEFT_X, -currentEvent.pose.getY());
            persistAxis(ControllerAxis.LEFT_Y, -currentEvent.pose.getX());
          }
          case MOVE_JOYSTICK_TURN -> {
            persistAxis(ControllerAxis.RIGHT_X, -currentEvent.pose.getY());
            persistAxis(ControllerAxis.RIGHT_Y, -currentEvent.pose.getX());
          }
          case STOP_JOYSTICK -> stopJoystick();
          case ENABLE_WHEEL_SLIP -> slipWheels = true;
          case DISABLE_WHEEL_SLIP -> slipWheels = false;
          case BLUE_INACTIVE_FIRST -> gameSpecificMessage = "B";
          case RED_INACTIVE_FIRST -> gameSpecificMessage = "R";
          case END_OF_SCENARIO -> {}
        }
      }
      if (eventIterator.hasNext()) {
        currentEvent = eventIterator.next();
      } else {
        // end of scenario
        currentEvent = null;
        if (events == autoEvents && !teleopEvents.isEmpty()) {
          events = teleopEvents;
          resetScenario();
        } else {
          setNextRegressTest();
        }
      }
    }

    // update Xbox controls
    int retry = 0;
    do {
      if (++retry > 5) {
        System.err.println(
            "*** Aborting sim: Xbox controller update retry limit exceeded at matchTime "
                + matchTimer.get());
        System.exit(1);
      }
      configureController();
      for (Map.Entry<Integer, Double> entry : axisValues.entrySet()) {
        DriverStationSim.setJoystickAxis(hidPort, entry.getKey(), entry.getValue());
      }
      DriverStationSim.setGameSpecificMessage(gameSpecificMessage);
      DriverStationSim.setJoystickPOV(hidPort, 0, activePOV);
      DriverStationSim.setJoystickButtons(hidPort, activeButtonBitmask);
      // notifyNewData() fails 1 out of 5000 calls for unknown reasons
      DriverStationSim.notifyNewData();
    } while (DriverStation.getStickAxisCount(hidPort) != 6
        || DriverStation.getStickButtonCount(hidPort) != 10);
    if (retry > 2) {
      System.out.println(
          "Warning: Xbox ontroller update required "
              + retry
              + " attempts at matchTime "
              + matchTimer.get());
    }
    releaseMomentaryButtons();
    if (momentaryPOV) {
      activePOV = POVDirection.NONE.value;
      momentaryPOV = false;
    }
  }

  private void configureController() {
    // ensure controller has expected number of axes and buttons
    DriverStationSim.setJoystickIsXbox(hidPort, true);
    DriverStationSim.setJoystickAxisCount(hidPort, 6);
    DriverStationSim.setJoystickButtonCount(hidPort, 10);
  }

  private void resetScenario() {
    DriverStationSim.setEnabled(false);
    disabledTimer.start();
    Logger.recordOutput("Sim/RegressionTest", "Disabled");
    Logger.recordOutput("Sim/Scenario", "Disabled");
    Logger.recordOutput("Sim/EventName", "Disabled");
    Logger.recordOutput("Sim/EventType", "Disabled");

    DriverStationSim.setAutonomous(events == autoEvents);
    if (currentAlliance == Alliance.Red) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Red2);
    } else {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue2);
    }

    eventIterator = events.iterator();
    if (events == autoEvents) {
      currentScenario = autoScenario.toString();
    } else {
      currentScenario = teleopScenario.toString();
    }
  }

  private void setNextRegressTest() {
    // reset all controls
    // don't call DriverStationSim.resetData() because it creates race conditions
    holdPOV(POVDirection.NONE);
    releaseTrigger(ControllerAxis.LEFT_TRIGGER);
    releaseTrigger(ControllerAxis.RIGHT_TRIGGER);
    stopJoystick();
    activeButtonBitmask = 0;
    momentaryButtonBitmask = 0;
    gameSpecificMessage = "";
    DriverStationSim.setEnabled(false);

    if (!regressionTestIterator.hasNext()) {
      System.out.println("All tests complete");
      System.exit(0);
    }
    currentRegressionTest = regressionTestIterator.next();
    autoScenario = currentRegressionTest.autoScenario;
    autoAnomalies = currentRegressionTest.autoAnomalies;
    teleopScenario = currentRegressionTest.teleopScenario;
    teleAnomalies = currentRegressionTest.teleAnomalies;
    currentAlliance = currentRegressionTest.alliance;
    autoEvents = buildAutoScenario();
    teleopEvents = buildTeleopScenario();
    if (!autoEvents.isEmpty()) {
      events = autoEvents;
    } else {
      events = teleopEvents;
    }
    drive.setPose(new Pose2d(0, 0, new Rotation2d()));
    resetScenario();
  }

  private void holdButton(XboxController.Button button) {
    activeButtonBitmask |= 1 << (button.value - 1);
  }

  private void releaseButton(XboxController.Button button) {
    activeButtonBitmask &= ~(1 << (button.value - 1));
  }

  private void pressButton(XboxController.Button button) {
    activeButtonBitmask |= 1 << (button.value - 1);
    momentaryButtonBitmask |= 1 << (button.value - 1);
  }

  private void releaseMomentaryButtons() {
    activeButtonBitmask &= ~momentaryButtonBitmask;
    momentaryButtonBitmask = 0;
  }

  private void holdPOV(POVDirection direction) {
    activePOV = direction.value;
  }

  private void pressPOV(POVDirection direction) {
    holdPOV(direction);
    momentaryPOV = true;
  }

  public void persistAxis(ControllerAxis axis, double value) {
    axisValues.put(axis.value, value);
  }

  private void stopJoystick() {
    persistAxis(ControllerAxis.LEFT_X, 0.0);
    persistAxis(ControllerAxis.LEFT_Y, 0.0);
    persistAxis(ControllerAxis.RIGHT_X, 0.0);
    persistAxis(ControllerAxis.RIGHT_Y, 0.0);
  }

  private void holdTrigger(ControllerAxis axis) {
    persistAxis(axis, 1.0);
  }

  private void releaseTrigger(ControllerAxis axis) {
    persistAxis(axis, 0.0);
  }

  public static boolean wheelSlip() {
    return slipWheels;
  }

  public static AutoName getAutoScenario() {
    return autoScenario;
  }

  private void configureControllerTestBindings() {
    RobotContainer.controller
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Button X True");
                }));
    RobotContainer.controller
        .x()
        .onFalse(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Button X False");
                }));
    RobotContainer.controller
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Left POV True");
                }));
    RobotContainer.controller
        .povLeft()
        .onFalse(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Left POV False");
                }));
    RobotContainer.controller
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Left Bumper True");
                }));
    RobotContainer.controller
        .leftBumper()
        .onFalse(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Left Bumper False");
                }));
    RobotContainer.controller
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Right Bumper True");
                }));
    RobotContainer.controller
        .rightBumper()
        .onFalse(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Right Bumper False");
                }));
    RobotContainer.controller
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Left Trigger True");
                }));
    RobotContainer.controller
        .leftTrigger()
        .onFalse(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Left Trigger False");
                }));
    RobotContainer.controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Right Trigger True");
                }));
    RobotContainer.controller
        .rightTrigger()
        .onFalse(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Right Trigger False");
                }));
  }
}
