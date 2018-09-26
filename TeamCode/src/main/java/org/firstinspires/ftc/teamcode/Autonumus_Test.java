package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Disabled
@Autonomous(name="Auto_test", group="Pushbot")

public class Autonumus_Test extends LinearOpMode {
    private config robot = new config(this.telemetry);

    @Override
        public void runOpMode (){
        robot.ConfigureRobtHardware(this.hardwareMap);

        robot.right_front.setPower(0.5);
        robot.left_front.setPower(-0.5);
        robot.right_back.setPower(0.5);
        robot.left_back.setPower(-0.5);

        sleep(3000);

        robot.right_front.setPower(0);
        robot.left_front.setPower(0);
        robot.left_back.setPower(0);
        robot.right_back.setPower(0);


    }


}
