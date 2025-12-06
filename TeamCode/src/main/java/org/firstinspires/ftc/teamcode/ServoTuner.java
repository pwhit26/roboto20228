package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTuner extends LinearOpMode {
    private Servo angleTurret0, angleTurret1;
    public DcMotorEx turret;
    private boolean rBumpLast, lBumpLast, aLast, aPressable;
    double skib1 = 0.5;
    double skib2 = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        rBumpLast = false;
        lBumpLast = false;
        turret=hardwareMap.get(DcMotorEx.class, "turret");
        angleTurret0 = hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret0.setPosition(skib1);
        angleTurret1.setPosition(skib2);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.right_bumper && !rBumpLast) {
                skib1 = skib1 + 0.01;
                skib2 = skib2 - 0.01;
            }
            rBumpLast = gamepad1.right_bumper;

            if (gamepad1.left_bumper && !lBumpLast) {
                skib1 = skib1 - 0.01;
                skib2 = skib2 + 0.01;
            }
            lBumpLast = gamepad1.left_bumper;

            angleTurret0.setPosition(skib1);
            angleTurret1.setPosition(skib2);
            
            telemetry.addLine("" + skib1);
            telemetry.addLine("" + skib2);
            telemetry.update();

            if (gamepad1.a && !aLast) {
                aPressable = !aPressable;
            }
            if (aPressable) {
                turret.setPower(-1);
                telemetry.addData("Turret Power", turret.getPower());
            }
            else {
                turret.setPower(0);
            }
            aLast = gamepad1.a;
        }
    }
}