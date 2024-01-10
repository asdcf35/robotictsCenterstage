package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotUtils.DriveTrain;

public class RobotHardware {

    public BNO055IMU imu;

//    public DcMotor motorCarousel;


    public DriveTrain driveTrain;


    public static final double WHEEL_DIAMETER = 6.0;
    public static final double DRIVE_MOTOR_TICKS_PER_ROTATION = 385.5; //changing from 537.6


    private HardwareMap hardwareMap;

    public RobotHardware(HardwareMap aHardwareMap, boolean initIMU) {
        hardwareMap = aHardwareMap;

        driveTrain = new DriveTrain(hardwareMap);

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

    public DcMotor motorR;
    public DcMotor motorL;

//    public DcMotor motorFR;
//    public DcMotor motorFL;
//    public DcMotor motorBR;
//    public DcMotor motorBL;
//    public DcMotor[] motors;

    public Servo airplaneLauncher;
    public static int AIRPLANE_LAUNCHER_LAUNCH_POS;

    // servo

    public static int LIFT_INTAKE_POS = 0;
    public static int LIFT_HOVER_POS = 125;
    public static int LIFT_LOW_POS = 250;
    public static int LIFT_MEDIUM_POS = 500;
    public static int LIFT_HIGH_POS = 1000;



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

//        public void telemetryUpdate(Telemetry telemetry) {
//            telemetry.addData("BL pos", motorBL.getCurrentPosition());
//            telemetry.addData("BR pos", motorBR.getCurrentPosition());
//            telemetry.addData("FR pos", motorFR.getCurrentPosition());
//            telemetry.addData("FL pos", motorFL.getCurrentPosition());
//            telemetry.addData("MotorL pos", motorL.getCurrentPosition());
//            telemetry.addData("MotorR pos", motorR.getCurrentPosition());
//        }

    }

        public void Intake() {
            motorL.setTargetPosition(LIFT_INTAKE_POS);
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL.setPower(1);
            motorR.setTargetPosition(LIFT_INTAKE_POS);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setPower(1);
            // servo intake pos
            // set intake on
            currentState = States.INTAKE;

        }

        public void Hover() {
            motorL.setTargetPosition(LIFT_INTAKE_POS);
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL.setPower(1);
            motorR.setTargetPosition(-LIFT_INTAKE_POS);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setPower(1);
            // servo hover pos
            // set intake off
            currentState = States.HOVER;
        }

        public void Low() {
            motorL.setTargetPosition(LIFT_LOW_POS);
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL.setPower(0.4);
            motorR.setTargetPosition(-LIFT_LOW_POS);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setPower(0.4);
            // servo low pos
            currentState = RobotHardware.States.LOW;
        }

        public void Medium() {
            motorL.setTargetPosition(LIFT_MEDIUM_POS);
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL.setPower(1);
            motorR.setTargetPosition(-LIFT_MEDIUM_POS);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setPower(1);
            // servo low pos
            currentState = RobotHardware.States.MEDIUM;
        }

        public void High() {
            motorL.setTargetPosition(LIFT_HIGH_POS);
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL.setPower(1);
            motorR.setTargetPosition(-LIFT_HIGH_POS);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setPower(1);
            // servo low pos
            currentState = RobotHardware.States.HIGH;
        }





    }

//    public void launch(){
//        airplaneLauncher.setPosition(AIRPLANE_LAUNCHER_LAUNCH_POS);
//    }
//}

