package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "zeroServo")
public class zeroServo extends LinearOpMode {
    private Servo servo1;
    private Servo servo2;
    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class, "popup");
        servo1.setPosition(1);
        //servo2 = hardwareMap.get(Servo.class, "angleTurret1");
        //servo2.setPosition(0.99);
        telemetry.addData("servo position:", servo1.getPosition());
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo1.setPosition(0.1);
                telemetry.addData("Servo Position: ", servo1.getPosition());
                //servo2.setPosition(0.9);
                //telemetry.addData("Servo Position: ", servo2.getPosition());
                telemetry.update();
            }
            else if (gamepad1.x) {
                servo1.setPosition(0.2);
                telemetry.addData("Servo Position: ", servo1.getPosition());
                //servo2.setPosition(0.8);
                //telemetry.addData("Servo Position: ", servo2.getPosition());
                telemetry.update();
            }
            else if (gamepad1.y) {
                servo1.setPosition(0.3);
                telemetry.addData("Servo Position: ", servo1.getPosition());
                //servo2.setPosition(0.75);
                //telemetry.addData("Servo Position: ", servo2.getPosition());
                telemetry.update();
            }
            else if (gamepad1.b) {
                servo1.setPosition(0.4);
                telemetry.addData("Servo Position: ", servo1.getPosition());
                //servo2.setPosition(1);
                //telemetry.addData("Servo Position: ", servo2.getPosition());
                telemetry.update();
            }
        }
    }
}
