package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotUtils.DriveTrain;

public class RobotHardware {

    public BNO055IMU imu;

//    public DcMotor motorCarousel;


    public DriveTrain driveTrain;
    public Lift lift;
    public static final double WHEEL_DIAMETER = 6.0;
    public static final double DRIVE_MOTOR_TICKS_PER_ROTATION = 385.5; //changing from 537.6


    private HardwareMap hardwareMap;

    public RobotHardware(HardwareMap aHardwareMap, boolean initIMU) {
        hardwareMap = aHardwareMap;

        driveTrain = new DriveTrain(hardwareMap);
        lift = new Lift(hardwareMap);

        if (initIMU) {
            initializeIMU();
        }


    }

    public void initializeIMU() {
        //------------------------------------------------------------
        // IMU - BNO055
        // Set up the parameters with which we will use our IMU.
        // + 9 degrees of freedom
        // + use of calibration file (see calibration program)
        //------------------------------------------------------------

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitImuCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        //parameters.loggingTag          = "IMU";
        //parameters.mode                = BNO055IMU.SensorMode.NDOF;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public States currentState = States.INTAKE;

    public DcMotor motorLiftR;
    public DcMotor motorLiftL;

//    public DcMotor motorFR;
//    public DcMotor motorFL;
//    public DcMotor motorBR;
//    public DcMotor motorBL;
//    public DcMotor[] motors;

    public Servo airplaneLauncher;
    public int AIRPLANE_LAUNCHER_LAUNCH_POS;

    // servo

    public int LIFT_INTAKE_POS = 0;
    public int LIFT_HOVER_POS = 125;
    public int LIFT_LOW_POS = 250;
    public int LIFT_MEDIUM_POS = 500;
    public int LIFT_HIGH_POS = 1000;


    // Robot states
    enum States {
        INTAKE,
        HOVER,
        LOW,
        MEDIUM,
        HIGH,

        HANG_INIT,
        HANG_TOP,
        HANG_GRAB,
    }

    public class DriveTrain {
        public DcMotor motorFL;
        public DcMotor motorFR;
        public DcMotor motorBL;
        public DcMotor motorBR;
        public DcMotor[] motors;


        public DriveTrain(HardwareMap hardwareMap) {
            motorFL = hardwareMap.get(DcMotor.class, "leftFront");
            motorFR = hardwareMap.get(DcMotor.class, "rightFront");
            motorBL = hardwareMap.get(DcMotor.class, "leftRear");
            motorBR = hardwareMap.get(DcMotor.class, "rightRear");
            motors = new DcMotor[]{motorFL, motorFR, motorBL, motorBR};


            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

            for (DcMotor motor : motors) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

        public void startMove(double drive, double strafe, double turn, double scale) {
            double powerFL = (drive + strafe + turn) * scale;
            double powerFR = (drive - strafe - turn) * scale;
            double powerBL = (drive - strafe + turn) * scale;
            double powerBR = (drive + strafe - turn) * scale;

            double maxPower = Math.max(Math.max(Math.abs(powerFL), Math.abs(powerFR)), Math.max(Math.abs(powerBL), Math.abs(powerBR)));
            double max = (maxPower < 1) ? 1 : maxPower;

            motorFL.setPower(Range.clip(powerFL / max, -1, 1));
            motorFR.setPower(Range.clip(powerFR / max, -1, 1));
            motorBL.setPower(Range.clip(powerBL / max, -1, 1));
            motorBR.setPower(Range.clip(powerBR / max, -1, 1));
        }

        public void stopMove() {
            for (DcMotor motor : motors) {
                motor.setPower(0);
            }
        }

        public void telemetryUpdate(Telemetry telemetry) {
            telemetry.addData("BL pos", motorBL.getCurrentPosition());
            telemetry.addData("BR pos", motorBR.getCurrentPosition());
            telemetry.addData("FR pos", motorFR.getCurrentPosition());
            telemetry.addData("FL pos", motorFL.getCurrentPosition());
            telemetry.addData("motorLiftL pos", motorLiftL.getCurrentPosition());
            telemetry.addData("motorLiftR pos", motorLiftR.getCurrentPosition());
        }

    }

    public void Intake() {
        motorLiftL.setTargetPosition(LIFT_INTAKE_POS);
        motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftL.setPower(0.4);
        motorLiftR.setTargetPosition(LIFT_INTAKE_POS);
        motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftR.setPower(0.4);
        // servo intake pos
        // set intake on

        currentState = States.INTAKE;

    }

    public void Hover() {
        motorLiftL.setTargetPosition(LIFT_INTAKE_POS);
        motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftL.setPower(0.4);
        motorLiftR.setTargetPosition(-LIFT_INTAKE_POS);
        motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftR.setPower(0.4);
        // servo hover pos
        // set intake off
        currentState = States.HOVER;
    }

    public void Low() {
        motorLiftL.setTargetPosition(LIFT_LOW_POS);
        motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftL.setPower(0.4);
        motorLiftR.setTargetPosition(-LIFT_LOW_POS);
        motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftR.setPower(0.4);
        // servo low pos
        currentState = RobotHardware.States.LOW;
    }

    public void Medium() {
        motorLiftL.setTargetPosition(LIFT_MEDIUM_POS);
        motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftL.setPower(0.4);
        motorLiftR.setTargetPosition(-LIFT_MEDIUM_POS);
        motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftR.setPower(0.4);
        // servo low pos
        currentState = RobotHardware.States.MEDIUM;
    }

    public void High() {
        motorLiftL.setTargetPosition(LIFT_HIGH_POS);
        motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftL.setPower(1);
        motorLiftR.setTargetPosition(-LIFT_HIGH_POS);
        motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftR.setPower(1);
        // servo low pos
        currentState = RobotHardware.States.HIGH;
    }


    public class Lift {
        public DcMotor motorLiftL;

        public DcMotor motorTurret;
        public DcMotor motorLiftR;
        public DcMotor motorIntake;

        public Servo servoClaw;

        public Servo servoExtension;
        public Object currentState;


        ElapsedTime elapsedTime;

        public  int LIFT_HIGH_POS = 3100;
        public int LIFT_MID_POS = 2150;
        public int LIFT_LOW_POS = 1250;
        public int LIFT_HOVER_POS = 100;
        public int LIFT_INTAKE_POS = 0;

        public int TURRET_LEFT_POS = 1545;
        public int TURRET_RIGHT_POS = -1560;
        public int TURRET_LEFT_135_POS = 2300;
        public int TURRET_RIGHT_135_POS = -2350;
        public int TURRET_LEFT_45_POS = 770;
        public int TURRET_RIGHT_45_POS = -780;
        public int TURRET_STACK_POS = -2700;
        public int TURRET_180_POS = 3080;
        public int TURRET_0_POS = 0;

        public double CLAW_CLOSE_POS = 0.52;
        public double CLAW_OPEN_POS = 0.72;
        public double CLAW_INIT_POS = 0.78;

        public double EXTENSION_SCORE_L_POS = 0.39;
        public double EXTENSION_SCORE_R_POS = 0.39;
        public double EXTENSION_180_SCORE_POS = 0.52;
        public double EXTENSION_INTAKE_POS = 0.26;
        public double EXTENSION_INIT_POS = 0.26;

        public double EXTENSION_INTAKE_OUT_POS = 0.6;
        public double EXTENSION_SCORE_45_POS = 0.52;
        public double EXTENSION_AUTO_POS = 0.6;
        public double EXTENSION_90_INTAKE_POS = 0.52;
        public double EXTENSION_90_AUTO_POS = 0.34;
        public double EXTENSION_90_AUTO_LOW_POS = 0.36;


        public Lift(HardwareMap hardwareMap) {
            motorLiftL = hardwareMap.get(DcMotor.class, "motorLiftL");

            motorLiftL.setDirection(DcMotorSimple.Direction.REVERSE);
            motorLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorLiftR = hardwareMap.get(DcMotor.class, "motorLiftR");

            motorLiftR.setDirection(DcMotorSimple.Direction.FORWARD);
            motorLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorTurret = hardwareMap.get(DcMotor.class, "motorTurret");

            motorTurret.setDirection(DcMotorSimple.Direction.FORWARD);
            motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            servoClaw = hardwareMap.get(Servo.class, "servoClaw");
            servoClaw.setPosition(CLAW_INIT_POS);

            servoExtension = hardwareMap.get(Servo.class, "servoExtension");
            servoExtension.setPosition(EXTENSION_INIT_POS);

            motorIntake = hardwareMap.get(DcMotor.class, "intake");
//            ElapsedTime elapsedTime = new ElapsedTime();

        }
    }

//    public void launch(){
//        airplaneLauncher.setPosition(AIRPLANE_LAUNCHER_LAUNCH_POS);
//    }
//}
}
