package frc.robot;

public class Constants {
    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 0;
    
        public static final double LinCoef = 0.2;
        public static final double Threshold = -0.0;//0.02;
        public static final double ZeroValue = 0.0;
        public static final double CuspX = 0.9;
        public static final double SpeedLimitX = 0.4;
        public static final double SpeedLimitRot = 0.4;
    }

  /**
   * Vision system constants (Coral robot only - single camera).
   */
  public static final class VisionConstants {
    /** PhotonVision camera name (must match name in PhotonVision UI) */
    public static final String CAMERA_NAME = "Logitech_Webcam_C930e";

    /** P-controller gain for rotation alignment */
    public static final double ROTATION_P = 0.05;

    /** P-controller gain for forward drive control */
    public static final double DRIVE_P = 0.1;

    /** Angle tolerance for alignment completion (degrees) */
    public static final double ANGLE_TOLERANCE = 2.0;

    /** Target area percentage for desired distance (~1.5-2m away) */
    public static final double AREA_TARGET = 8.0;

    /** Area tolerance for distance completion */
    public static final double AREA_TOLERANCE = 1.0;

    /** Minimum rotation speed (rad/s) */
    public static final double MIN_ROTATION_SPEED = 0.1;

    /** Maximum rotation speed (rad/s) */
    public static final double MAX_ROTATION_SPEED = 2.0;

    /** Minimum drive speed (m/s) */
    public static final double MIN_DRIVE_SPEED = 0.2;

    /** Maximum drive speed (m/s) */
    public static final double MAX_DRIVE_SPEED = 1.5;

    /** Maximum yaw error before stopping forward drive (degrees) */
    public static final double MAX_YAW_ERROR_FOR_DRIVE = 15.0;
  }
}
