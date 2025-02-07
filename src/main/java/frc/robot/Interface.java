package frc.robot;

public class Interface {
    public class Bling {
        static final int kNoteSignal = Ports.kDIO_4;
    }

    public class Climb {
        static final int kWinchMotor = Ports.kCAN_61;

        static final int kLockServo = Ports.kPWM_1;

        static final int kLimitLeft = Ports.kDIO_2;
        static final int kLimitRight = Ports.kDIO_3;
    }

    public class Drive {
        public static final int kBackRightDrive = Ports.kCAN_01;
        public static final int kBackRightTurn = Ports.kCAN_05;
        public static final int kBackRightEncoder = Ports.kCAN_21;

        public static final int kFrontRightDrive = Ports.kCAN_02;
        public static final int kFrontRightTurn = Ports.kCAN_06;
        public static final int kFrontRightEncoder = Ports.kCAN_22;

        public static final int kBackLeftDrive = Ports.kCAN_03;
        public static final int kBackLeftTurn = Ports.kCAN_07;
        public static final int kBackLeftEncoder = Ports.kCAN_23;

        public static final int kFrontLeftDrive = Ports.kCAN_04;
        public static final int kFrontLeftTurn = Ports.kCAN_08;
        public static final int kFrontLeftEncoder = Ports.kCAN_24;
    }

    public class Intake {
        static final int kMotorBottom = Ports.kCAN_41;
        static final int kMotorTop = Ports.kCAN_42;
    }

    public class Robot {
        static final int kRobotId = Ports.kDIO_9;
    }

    public class Shooter {
        static final int kMotor1 = Ports.kCAN_31;
        static final int kMotor2 = Ports.kCAN_32;

        static final int kFeedMotor = Ports.kCAN_33;

        static final int kDistanceSensorLeft = Ports.kCAN_34;
        static final int kDistanceSensorRight = Ports.kCAN_35;

        static final int kNoteSensorBottom = Ports.kDIO_0;
        static final int kNoteSensorTop = Ports.kDIO_1;
    }

    public class Tilt {
        static final int kTiltLeft = Ports.kCAN_36;
        static final int kTiltRight = Ports.kCAN_37;

        static final int kTiltSensor = Ports.kAI_1;
    }

    private class Ports {
        static final int kAI_0 = 0;
        static final int kAI_1 = 1;

        // Drive
        static final int kCAN_01 = 1;
        static final int kCAN_02 = 2;
        static final int kCAN_03 = 3;
        static final int kCAN_04 = 4;

        static final int kCAN_05 = 5;
        static final int kCAN_06 = 6;
        static final int kCAN_07 = 7;
        static final int kCAN_08 = 8;

        static final int kCAN_21 = 21;
        static final int kCAN_22 = 22;
        static final int kCAN_23 = 23;
        static final int kCAN_24 = 24;

        // Shooter
        static final int kCAN_31 = 31;
        static final int kCAN_32 = 32;
        static final int kCAN_33 = 33;
        static final int kCAN_34 = 34;
        static final int kCAN_35 = 35;
        static final int kCAN_36 = 36;
        static final int kCAN_37 = 37;

        // Intake
        static final int kCAN_41 = 41;
        static final int kCAN_42 = 42;

        // Climb
        static final int kCAN_61 = 61;

        // DIO

        static final int kDIO_0 = 0;
        static final int kDIO_1 = 1;
        static final int kDIO_2 = 2;
        static final int kDIO_3 = 3;
        static final int kDIO_4 = 4;

        static final int kDIO_9 = 9;

        // PWM

        static final int kPWM_1 = 1;
    }
}
