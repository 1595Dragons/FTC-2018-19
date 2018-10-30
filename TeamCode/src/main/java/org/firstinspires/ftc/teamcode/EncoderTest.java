package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Stephen Ogden on 10/30/18.
 * FTC 6128 | 7935
 * FRC 1595
 */
@TeleOp(name = "Encoder test", group = "Test")
public class EncoderTest extends LinearOpMode {

    public void runOpMode() {

        // Update telemetry that robot is initializing...
        telemetry.addData("Status", "Initializing robot. Please wait...");
        telemetry.update();

        DcMotor left_front = this.hardwareMap.dcMotor.get("1");
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Motor 1/4 ready");
        telemetry.update();

        DcMotor right_front = this.hardwareMap.dcMotor.get("2");
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Motor 2/4 ready");
        telemetry.update();

        DcMotor left_back = this.hardwareMap.dcMotor.get("3");
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Motor 3/4 ready");
        telemetry.update();

        DcMotor right_back = this.hardwareMap.dcMotor.get("4");
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Motor 4/4 ready");
        telemetry.update();

        telemetry.addData("Status", "Ready!");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("1", left_front.getCurrentPosition())
                    .addData("2", right_front.getCurrentPosition())
                    .addData("3", left_back.getCurrentPosition())
                    .addData("4", right_back.getCurrentPosition());
            telemetry.update();

        }
    }
}
