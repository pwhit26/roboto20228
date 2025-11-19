package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTuner extends LinearOpMode {
    private Servo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        while (opModeIsActive()){

        }
    }
}