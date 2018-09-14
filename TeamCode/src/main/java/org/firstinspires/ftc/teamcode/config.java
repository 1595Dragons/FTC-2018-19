package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is a test config file for getting a rough mecanum drive working.
 * We're probably not going to use the for the official robot 😅
 *
 * Created by Stephen Ogden on 9/13/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
public class config {

    @SuppressWarnings("WeakerAccess")
    public DcMotor left_front, right_front, left_back, right_back;

    public void init(Telemetry telemetry, HardwareMap hardware) {

        telemetry.addData("Status", "Initializing robot. Please wait...");
        telemetry.update();

        left_front = hardware.dcMotor.get("left front");
        left_front.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        left_front.setMode(RunMode.RUN_WITHOUT_ENCODER);
        left_front.setDirection(Direction.FORWARD);

        right_front = hardware.dcMotor.get("right front");
        right_front.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        right_front.setMode(RunMode.RUN_WITHOUT_ENCODER);
        right_front.setDirection(Direction.REVERSE);

        left_back = hardware.dcMotor.get("left back");
        left_back.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        left_back.setMode(RunMode.RUN_WITHOUT_ENCODER);
        left_back.setDirection(Direction.FORWARD);

        right_back = hardware.dcMotor.get("right back");
        right_back.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        right_back.setMode(RunMode.RUN_WITHOUT_ENCODER);
        right_back.setDirection(Direction.REVERSE);

        telemetry.addData("Status", "Ready!");
        telemetry.update();

    }


}
