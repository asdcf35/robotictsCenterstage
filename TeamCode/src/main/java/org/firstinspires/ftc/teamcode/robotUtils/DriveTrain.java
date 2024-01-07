package org.firstinspires.ftc.teamcode.robotUtils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorBL;
    public DcMotor[] motors;
    enum States {
        ACTIVE, INACTIVE
    }
    public DriveTrain(HardwareMap hardwareMap) {
        // hardware map
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
    }
}
