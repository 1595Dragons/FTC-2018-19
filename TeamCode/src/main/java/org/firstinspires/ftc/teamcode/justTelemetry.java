package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Locale;

/**
 * Created by Stephen Ogden on 11/13/18.
 * FTC 6128 | 7935
 * FRC 1595
 */

@TeleOp(name = "Just telemetry", group = "Test")
public class justTelemetry extends LinearOpMode {
    Config robot = new Config(this);

    @Override
    public void runOpMode() {
        robot.ConfigureRobtHardware(true);

        // Create a list of all the devices used in the config, and add their names
        ArrayList<HardwareDevice> Devices = new ArrayList<>();
        ArrayList<String> DeviceNames = new ArrayList<>();

        // Add all the devices and their names
        Devices.add(robot.left_front);
        DeviceNames.add("Left front");
        Devices.add(robot.right_front);
        DeviceNames.add("Right front");
        Devices.add(robot.left_back);
        DeviceNames.add("Left back");
        Devices.add(robot.right_back);
        DeviceNames.add("Right back");
        Devices.add(robot.armMotorL);
        DeviceNames.add("Left arm motor");
        Devices.add(robot.armMotorR);
        DeviceNames.add("Right arm motor");
        Devices.add(robot.armMotorExtend);
        DeviceNames.add("Arm extension motor");
        Devices.add(robot.IO_Motor);
        DeviceNames.add("Intake motor");
        Devices.add(robot.IO_Servo_Left);
        DeviceNames.add("Left servo");
        Devices.add(robot.IO_Servo_Right);
        DeviceNames.add("Right servo");

        waitForStart();
        while (opModeIsActive()) {

            int i = 0;
            for (HardwareDevice device : Devices) {
                if (device != null) {
                    String deviceName = DeviceNames.get(i);
                    if (device instanceof DcMotor) {
                        DcMotor motor = (DcMotor) device;
                        telemetry.addData(deviceName + " power", String.format(Locale.US, "%.2f", motor.getPower()))
                                .addData(deviceName + " position", motor.getCurrentPosition())
                                .addData(deviceName + " target (Displacement)", String.format(Locale.US, "%s (%s)",
                                        motor.getTargetPosition(), Math.abs(motor.getCurrentPosition() - motor.getTargetPosition())));
                        telemetry.addLine();
                    } else if (device instanceof Servo) {
                        Servo servo = (Servo) device;
                        telemetry.addData(deviceName + " target position", servo.getPosition());
                        telemetry.addLine();
                    }
                }
                i++;
            }

            if (robot.imu != null) {
                if (robot.imu.isGyroCalibrated()) {
                    telemetry.addData("Robot rotation", robot.getAngle());
                    telemetry.addLine();
                }
            }


            if (robot.goldDetector != null) {
                if (robot.goldDetector.isFound()) {
                    telemetry.addData("Gold detector", "Found gold");
                    telemetry.addData("Gold location", robot.goldDetector.getFoundRect().x + ", " + robot.goldDetector.getFoundRect().y);
                    telemetry.addLine();
                } else {
                    telemetry.addData("Gold detector", "Still searching...");
                }
            }


            // Update the telemetry
            telemetry.update();


        }
    }
}
