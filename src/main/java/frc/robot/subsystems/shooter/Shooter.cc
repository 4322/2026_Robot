

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

class Shooter {
 public:
    // Create a shooter subsystem using a single brushless motor (NEO).
    // motorCanId: CAN ID of the Spark Max controlling the shooter motor.
    explicit Shooter(int motorCanId = 1)
            : m_motor{motorCanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                m_encoder{m_motor.GetEncoder()},
                m_pid{m_motor.GetPIDController()},
                m_targetRPM{0.0} {
        // Basic motor configuration
        m_motor.RestoreFactoryDefaults();
        m_motor.SetSmartCurrentLimit(40);  // amps, tune as needed
        m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        m_motor.SetInverted(false);

        // Encoder & PID defaults (tune these values for your robot)
        m_pid.SetP(0.0005);
        m_pid.SetI(0.0);
        m_pid.SetD(0.0);
        m_pid.SetFF(0.000175);
        m_pid.SetOutputRange(-1.0, 1.0);

        // Publish initial values
        frc::SmartDashboard::PutNumber("Shooter/TargetRPM", 0.0);
        frc::SmartDashboard::PutNumber("Shooter/MeasuredRPM", 0.0);
    }

    ~Shooter() = default;

    // Set shooter target using closed-loop RPM control
    void SetRPM(double rpm) {
        m_targetRPM = rpm;
        // SparkMax PID expects velocity in RPM when using kVelocity
        m_pid.SetReference(rpm, rev::ControlType::kVelocity);
        frc::SmartDashboard::PutNumber("Shooter/TargetRPM", rpm);
    }

    // Open-loop percent output [-1..1]
    void SetPercent(double percent) {
        m_targetRPM = 0.0;
        m_motor.Set(percent);
        frc::SmartDashboard::PutNumber("Shooter/TargetPercent", percent);
    }

    // Stop the shooter motor
    void Stop() {
        m_targetRPM = 0.0;
        m_motor.StopMotor();
        frc::SmartDashboard::PutNumber("Shooter/TargetRPM", 0.0);
        frc::SmartDashboard::PutNumber("Shooter/TargetPercent", 0.0);
    }

    // Returns current measured RPM from the encoder
    double GetRPM() const { return m_encoder.GetVelocity(); }

    // Periodic-like update for telemetry
    void Periodic() {
        frc::SmartDashboard::PutNumber("Shooter/MeasuredRPM", GetRPM());
    }

 private:
    rev::CANSparkMax m_motor;
    rev::SparkMaxRelativeEncoder m_encoder;
    rev::SparkMaxPIDController m_pid;
    double m_targetRPM;
};
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

class Shooter : public frc2::SubsystemBase {
 public:
    // Create a shooter subsystem using a single brushless motor (NEO).
    // motorCanId: CAN ID of the Spark Max controlling the shooter motor.
    explicit Shooter(int motorCanId = 1)
            : m_motor{motorCanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                m_encoder{m_motor.GetEncoder()},
                m_pid{m_motor.GetPIDController()},
                m_targetRPM{0.0} {
        // Basic motor configuration
        m_motor.RestoreFactoryDefaults();
        m_motor.SetSmartCurrentLimit(40);  // amps, tune as needed
        m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        m_motor.SetInverted(false);

        // Encoder & PID defaults (tune these values for your robot)
        m_pid.SetP(0.0005);
        m_pid.SetI(0.0);
        m_pid.SetD(0.0);
        m_pid.SetFF(0.000175);
        m_pid.SetOutputRange(-1.0, 1.0);

        // Publish initial values
        frc::SmartDashboard::PutNumber("Shooter/TargetRPM", 0.0);
        frc::SmartDashboard::PutNumber("Shooter/MeasuredRPM", 0.0);
    }

    ~Shooter() override = default;

    // Set shooter target using closed-loop RPM control
    void SetRPM(double rpm) {
        m_targetRPM = rpm;
        // SparkMax PID expects velocity in RPM when using kVelocity
        m_pid.SetReference(rpm, rev::ControlType::kVelocity);
        frc::SmartDashboard::PutNumber("Shooter/TargetRPM", rpm);
    }

    // Open-loop percent output [-1..1]
    void SetPercent(double percent) {
        m_targetRPM = 0.0;
        m_motor.Set(percent);
        frc::SmartDashboard::PutNumber("Shooter/TargetPercent", percent);
    }

    // Stop the shooter motor
    void Stop() {
        m_targetRPM = 0.0;
        m_motor.StopMotor();
        frc::SmartDashboard::PutNumber("Shooter/TargetRPM", 0.0);
        frc::SmartDashboard::PutNumber("Shooter/TargetPercent", 0.0);
    }

    // Returns current measured RPM from the encoder
    double GetRPM() const { return m_encoder.GetVelocity(); }

    // Periodic called by scheduler; update telemetry
    void Periodic() override {
        frc::SmartDashboard::PutNumber("Shooter/MeasuredRPM", GetRPM());
    }

 private:
    rev::CANSparkMax m_motor;
    rev::SparkMaxRelativeEncoder m_encoder;
    rev::SparkMaxPIDController m_pid;
    double m_targetRPM;
};