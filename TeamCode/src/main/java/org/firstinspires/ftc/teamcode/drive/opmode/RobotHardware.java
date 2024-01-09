package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotUtils.DriveTrain;

public class RobotHardware {
    public DriveTrain drivetrain;


    public States INTAKE;

    public DcMotor motorR;
    public DcMotor motorL;

    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorBL;
    public DcMotor[] motors;

    public Servo airplaneLauncher;
    public static int AIRPLANE_LAUNCHER_LAUNCH_POS;

    // servo

    public static int LIFT_INTAKE_POS = 0;
    public static int LIFT_HOVER_POS = 125;
    public static int LIFT_LOW_POS = 250;
    public static int LIFT_MEDIUM_POS = 200;
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
    public States currentState;

    public RobotHardware(HardwareMap hardwareMap) {
        // hardware map
        motorL = hardwareMap.get(DcMotor.class, "leftSlides");

        motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorR = hardwareMap.get(DcMotor.class, "rightSlides");

        motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR = hardwareMap.dcMotor.get("rightRear");
        motorFL = hardwareMap.dcMotor.get("leftRear");
        motorBR = hardwareMap.dcMotor.get("rightFront");
        motorBL = hardwareMap.dcMotor.get("leftFront");
        motors = new DcMotor[] {motorFR, motorFL, motorBR, motorBL};

        // Set directions for motors that require a change
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        airplaneLauncher = hardwareMap.get(Servo.class,"airplaneLauncher");
    }

        public void Intake () {
            motorL.setTargetPosition(LIFT_INTAKE_POS);
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL.setPower(0.5);
            motorR.setTargetPosition(-LIFT_INTAKE_POS);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setPower(0.5);
            // servo intake pos
            // set intake on

        }

        public void Hover () {
            motorL.setTargetPosition(LIFT_INTAKE_POS);
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL.setPower(0.5);
            motorR.setTargetPosition(-LIFT_INTAKE_POS);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setPower(0.5);
            // servo hover pos
            // set intake off
        }
        public void Low () {
            motorL.setTargetPosition(LIFT_LOW_POS);
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL.setPower(0.5);
            motorR.setTargetPosition(-LIFT_LOW_POS);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setPower(0.5);
            // servo low pos
        }
        public void Medium () {
            motorL.setTargetPosition(LIFT_MEDIUM_POS);
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL.setPower(0.5);
            motorR.setTargetPosition(-LIFT_MEDIUM_POS);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setPower(-0.5);
            // servo low pos
        }
        public void High () {
            motorL.setTargetPosition(LIFT_HIGH_POS);
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorL.setPower(0.5);
            motorR.setTargetPosition(-LIFT_HIGH_POS);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setPower(0.5);
            // servo low pos
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
    public void telemetryUpdate(Telemetry telemetry) {
        telemetry.addData("BL pos", motorBL.getCurrentPosition());
        telemetry.addData("BR pos", motorBR.getCurrentPosition());
        telemetry.addData("FR pos", motorFR.getCurrentPosition());
        telemetry.addData("FL pos", motorFL.getCurrentPosition());
        telemetry.addData("L pos",motorL.getCurrentPosition());
        telemetry.addData("R pos", motorR.getCurrentPosition());
    }
    public void setAirplaneLauncher(HardwareMap hardwareMap){
    }
    public void launch(){
        airplaneLauncher.setPosition(AIRPLANE_LAUNCHER_LAUNCH_POS);
    }
}

