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
    config robot = new config();

    public DcMotor right;
    public DcMotor left;
    public DcMotor right_2;
    public DcMotor left_2;
    @Override
        public void runOpMode (){
            robot.init(this.telemetry, this.hardwareMap);
        right = hardwareMap.dcMotor.get("right front");
        left = hardwareMap.dcMotor.get("left front");
        right_2 = hardwareMap.dcMotor.get("right back");
        left_2 = hardwareMap.dcMotor.get("left back");

        right.setPower(0.5);
        left.setPower(-0.5);
        right_2.setPower(0.5);
        left_2.setPower(-0.5);

        sleep(3000);

        right.setPower(0);
        left.setPower(0);
        right_2.setPower(0);
        left_2.setPower(0);


    }


}
