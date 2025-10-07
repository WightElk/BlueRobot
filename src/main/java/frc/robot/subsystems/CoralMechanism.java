package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralMechanism extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMax sideMotor;
  private boolean motorOn = false;
  private final CANBus kCANBus;
  private final CANdi candi;

  public CoralMechanism(CANBus canBus) {
    kCANBus = canBus;
    candi = new CANdi(7, kCANBus);
    motor = new SparkMax(3, MotorType.kBrushless); // CAN ID 3

    sideMotor = new SparkMax(4, MotorType.kBrushless); // CAN ID 3

    SparkMaxConfig config = new SparkMaxConfig();
    // config.voltageCompensation(ArmConstants.ARM_MOTOR_VOLTAGE_COMP);
    //   config.smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
    //   config.idleMode(IdleMode.kCoast);
    //motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(3, true);
    sideMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        /* Configure CANdi */
    var toApply = new CANdiConfiguration();

    toApply.DigitalInputs.S1FloatState = S1FloatStateValue.FloatDetect; // Pulse-width sensor will drive low. Default of FloatDetect will typically work on most sensors.
    toApply.DigitalInputs.S2FloatState = S2FloatStateValue.FloatDetect;
    toApply.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenLow; // This example specifically assumes a hardware limit switch will close S2 to Ground. Default of CloseWhenNotFloating will also work
    toApply.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow;

//    toApply.PWM1.SensorDirection = true; // Invert the PWM1 position.
//    toApply.PWM1.AbsoluteSensorDiscontinuityPoint = 0.75; // If the PWM 1 position on boot is after 0.75 rotations, treat it as x - 1 rotations.
                                                          // As an example, if the position is 0.87, it will boot to 0.87 - 1 = -0.13 rotations.
    /* User can change the configs if they want, or leave it empty for factory-default */
    candi.getConfigurator().apply(toApply);
      /* Speed up signals to an appropriate rate */
//    BaseStatusSignal.setUpdateFrequencyForAll(100, candi.getPWM1Position(), candi.getPWM1Velocity(), candi.getS2State());

    // Add a SmartDashboard slider for manual control
    SmartDashboard.putNumber("Coral Motor Speed", 0.0);
  }
  boolean prev = true;
  boolean inside = false;
  boolean outside = false;
  
  boolean prevActivated1 = false;
  boolean prevActivated2 = false;
  int stage = 1;

  public void reset() {
    System.out.println("CoralIn Reset ");
    prev = true;
    inside = outside = false;
    prevActivated1 = false;
    prevActivated2 = false;
    stage = stage == 1 ? 0 : 1;

    var s2Closed = candi.getS2Closed();
    boolean activated2 = !s2Closed.getValue();
    prevActivated2 = activated2;
    stage = activated2 ? 1 : 0;
  }

  public boolean isCoralIn() {
      //boolean return !candprevActivated1 = false;i.getS2Closed().getValue();
      return inside || outside;
  }

  public void setSpeed1(double speed) {
    motor.set(speed);
  }

  public void setSpeed(double speed, boolean into) {
    var s1Closed = candi.getS1Closed();
    var s2Closed = candi.getS2Closed();
    boolean activated1 = !s1Closed.getValue();
    boolean activated2 = !s2Closed.getValue();
    //boolean activated2 = !s2Closed.getValue();

    if (stage == 1) {
      outside = !activated2 && prevActivated2;

      speed = speed > 0 ? 1 : speed;
      motor.set(-speed);
    }
    else {
      inside = !activated1 && prevActivated1;
      // if (inside)
      //   stage = 1;

        speed = speed > 0 ? 0.5 : speed;
        double s = !activated1 ? speed : 0.5 * speed;
      motor.set(-s);
    }

    prevActivated1 = activated1;
    prevActivated2 = activated2;

    // if (into) {
    //   boolean curr = s1Closed.getValue();
    //   inside = !prev && curr;
    //   double s = curr ? speed : 0.5 * speed;
    //   //if (!isCoralIn())
    //   if (inside)
    //       s = 0;
    //   s *= 0.8;
    //   System.out.println("CoralIn Speed " + s);
    //   motor.set(-s);
    //   prev = curr;
    // }
    // else {
    //   prev = true;
    //   inside = false;
    //   speed = 0.5;
    //   motor.set(-speed);
    // }
    SmartDashboard.putNumber("Coral Motor Applied Output", motor.getAppliedOutput());
  }

  @Override
  public void periodic() {
          /**
       * Get the S2 State StatusSignalValue without refreshing
       */
      var s1Closed = candi.getS1Closed();
      var s2Closed = candi.getS2Closed();
      var s1State = candi.getS1State(true);
      var s2State = candi.getS2State(true);
      
      /* This time wait for the signal to reduce latency */
//      s2State.waitForUpdate(PRINT_PERIOD); // Wait up to our period
      /**
       * This uses the explicit getValue and getUnits functions to print, even though it's not
       * necessary for the ostream print
       */

      // s1Closed = true when OPEN!!! and s1State.getValue() = Low
      // FACT: < 0.005 sec latency
      // System.out.println(
      //   "S1 State is " + s1Closed + " " +
      //   s1State.getValue() + " " +
      //   s1State.getUnits() + " with " +
      //   s1State.getTimestamp().getLatency() + " seconds of latency"
      // );
      // System.out.println(
      //   "S2 State is " + s2Closed + " " +
      //   s2State.getValue() + " " +
      //   s2State.getUnits() + " with " +
      //   s2State.getTimestamp().getLatency() + " seconds of latency"
      // );

    // Optional: allow manual override from dashboard
    double manualSpeed = SmartDashboard.getNumber("Coral Motor Speed", 0.0);
    if (manualSpeed != 0.0) {
//      double speed = s2Closed.getValue() ? manualSpeed : 0.0;
//      if (s2Closed.getValue())
//      System.out.println("Speed " + speed);
      motor.set(manualSpeed);
    }
  }
}
