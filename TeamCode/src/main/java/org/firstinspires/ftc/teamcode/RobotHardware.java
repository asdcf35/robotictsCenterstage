package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {


    public BNO055IMU imu;

//    public DcMotor motorCarousel;


    public DriveTrain driveTrain;
    public Lift lift;
    public static final double WHEEL_DIAMETER = 6.0;
    public static final double DRIVE_MOTOR_TICKS_PER_ROTATION = 385.5; //changing from 537.6


    private HardwareMap hardwareMap;

    public RobotHardware(HardwareMap aHardwareMap, boolean initIMU, boolean initDT, boolean initLift, boolean initAirplane) {
        hardwareMap = aHardwareMap;
        if(initDT) {
            driveTrain = new DriveTrain(hardwareMap);
        }
        if(initLift) {
            lift = new Lift(hardwareMap);
        }
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
        parameters.calibrationDataFile = "AdafruitImuCalibration.json";
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

    public Servo airplaneLauncher;
    public int AIRPLANE_LAUNCHER_LAUNCH_POS = 250;

    // servo




    // Robot states
    enum States {
        INTAKE, HOVER, LOW, MEDIUM, HIGH,

        HANG_INIT, HANG_TOP, HANG_GRAB,
    }

    public class Lift {
        private static final int HANG_INIT_POS = 0;
        private static final int HANG_TOP_POS = 1000;
        private static final int HANG_GRAB_POS = 250;
        public DcMotor motorLift;
        public DcMotor motorIntakeHighSpeed;
        public DcMotor motorIntakeLowSpeed;
        public DcMotor motorHang;
        public Servo servoIntake;
        public Servo servoLauncher;
        public Servo servoBucket;
        public int LIFT_INTAKE_POS = 0;
        public int LIFT_HOVER_POS = 100;
        public int LIFT_LOW_POS = 2350;
        public int LIFT_MEDIUM_POS = 3350;
        public int LIFT_HIGH_POS = 2000;

        public double INTAKE_HS_POWER = 0.7;

        public double INTAKE_LS_POWER = 0.9;

        public double SERVO_INTAKE_INIT_POS = 0.8;
        public double SERVO_INTAKE_REG_POS = 0.29;
        public double SERVO_INTAKE_STACK_POS = 5;
        public double SERVO_INTAKE_HOVER_POS = 0.35;
        public double SERVO_INTAKE_JAM_POS = 0.265;

        public double SERVO_LAUNCHER_INT_POS = 0.25;
        public double SERVO_LAUNCHER_LAUNCH_POS = 0.75;

        public double SERVO_BUCKET_INIT_POS = 0.47;
        public double SERVO_BUCKET_INTAKE_POS = 0.47;
        public double SERVO_BUCKET_OUTAKE_POS = 0.12; // was 0.15

        public Lift(HardwareMap hardwareMap) {
            motorIntakeHighSpeed = hardwareMap.get(DcMotor.class, "intakeHS");
            motorIntakeHighSpeed.setDirection(DcMotorSimple.Direction.REVERSE);
            motorIntakeHighSpeed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorIntakeHighSpeed.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorIntakeHighSpeed.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorIntakeLowSpeed = hardwareMap.get(DcMotor.class, "intakeLS");
            motorIntakeLowSpeed.setDirection(DcMotorSimple.Direction.REVERSE);
            motorIntakeLowSpeed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorIntakeLowSpeed.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorIntakeLowSpeed.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorLift = hardwareMap.get(DcMotor.class, "motorLift");
            motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
            motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            servoIntake = hardwareMap.get(Servo.class, "servoIntake");
            servoIntake.setPosition(SERVO_INTAKE_INIT_POS);

            servoLauncher = hardwareMap.get(Servo.class, "servoLauncher");
            servoLauncher.setPosition(SERVO_LAUNCHER_INT_POS);

            servoBucket = hardwareMap.get(Servo.class, "servoBucket");
            servoBucket.setPosition(SERVO_BUCKET_INIT_POS);
        }
    }
    public class DriveTrain {
        public DcMotor motorFL;
        public DcMotor motorFR;
        public DcMotor motorBL;
        public DcMotor motorBR;
        public DcMotor[] motors;


        public DriveTrain(HardwareMap hardwareMap) {
            motorFL = hardwareMap.get(DcMotor.class, "FLmotor");
            motorFR = hardwareMap.get(DcMotor.class, "FRmotor");
            motorBL = hardwareMap.get(DcMotor.class, "BLmotor");
            motorBR = hardwareMap.get(DcMotor.class, "BRmotor");
            motors = new DcMotor[]{motorFL, motorFR, motorBL, motorBR};


            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
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

        public void resetDriveEncoders() {
            motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void stopMove() {
            for (DcMotor motor : motors) {
                motor.setPower(0);
            }
        }
    }

    public void telemetryUpdate(Telemetry telemetry) {
        telemetry.addData("BL pos", driveTrain.motorBL.getCurrentPosition());
        telemetry.addData("BR pos", driveTrain.motorBR.getCurrentPosition());
        telemetry.addData("FR pos", driveTrain.motorFR.getCurrentPosition());
        telemetry.addData("FL pos", driveTrain.motorFL.getCurrentPosition());
        telemetry.addData("motorLift pos", lift.motorLift.getCurrentPosition());
        //telemetry.addData("motorHang pos", lift.motorHang.getCurrentPosition());
    }


    public void Intake() {
        lift.motorLift.setTargetPosition(lift.LIFT_INTAKE_POS);
        lift.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.motorLift.setPower(1);
        lift.servoIntake.setPosition(lift.SERVO_INTAKE_REG_POS);
        currentState = States.INTAKE;
    }

    public void turnOnIntake() {
        lift.motorIntakeHighSpeed.setPower(lift.INTAKE_HS_POWER);
        lift.motorIntakeLowSpeed.setPower(lift.INTAKE_LS_POWER);
    }

    public void Hover() {
        lift.motorLift.setTargetPosition(lift.LIFT_HOVER_POS);
        lift.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.motorLift.setPower(1);
        lift.servoIntake.setPosition(lift.SERVO_INTAKE_REG_POS);

        currentState = States.HOVER;
    }

    private void turnOffIntake() {
        lift.motorIntakeHighSpeed.setPower(0);
        lift.motorIntakeLowSpeed.setPower(0);
    }

    public void Low() {
        lift.motorLift.setTargetPosition(lift.LIFT_LOW_POS);
        lift.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.motorLift.setPower(1);
        turnOffIntake();
        currentState = RobotHardware.States.LOW;
    }

    public void Medium() {
        lift.motorLift.setTargetPosition(lift.LIFT_MEDIUM_POS);
        lift.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.motorLift.setPower(1);
        turnOffIntake();
        currentState = RobotHardware.States.MEDIUM;
    }

    public void High() {
        lift.motorLift.setTargetPosition(lift.LIFT_HIGH_POS);
        lift.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.motorLift.setPower(0.5);
        turnOffIntake();
        currentState = RobotHardware.States.HIGH;
    }

    /*
     * Hang Functions Belong Here
     * */
    public void HangInit() {
        lift.motorLift.setTargetPosition(Lift.HANG_INIT_POS);
        lift.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.motorLift.setPower(0.4);
        currentState = States.INTAKE;
    }

    public void HangGrab() {
        lift.motorLift.setTargetPosition(-Lift.HANG_INIT_POS);
        lift.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.motorLift.setPower(0.4);
        currentState = States.INTAKE;
    }

    public void HangTop() {
        lift.motorHang.setTargetPosition(-Lift.HANG_INIT_POS);
        lift.motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.motorHang.setPower(-0.4);
        currentState = States.HANG_TOP;
    }

    public void HangTopRev() {
        lift.motorHang.setTargetPosition(Lift.HANG_INIT_POS);
        lift.motorHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.motorHang.setPower(0.4);
        currentState = States.HANG_TOP;
    }


}
