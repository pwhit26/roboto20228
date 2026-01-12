package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTuner extends LinearOpMode {
    private Servo angleTurret0, angleTurret1;
    public DcMotorEx turret;
    private boolean rBumpLast, lBumpLast, aLast, aPressable;
    double skib0 = 0;
    double skib1 = 1;
    long startTime = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        rBumpLast = false;
        lBumpLast = false;
        turret=hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angleTurret0 = hardwareMap.get(Servo.class, "popup");
        //angleTurret1 = hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret0.setPosition(skib0);
        //angleTurret1.setPosition(skib1);
        waitForStart();
        while (opModeIsActive()){
            long elapsedTime = System.currentTimeMillis() - startTime;
            if (gamepad1.right_bumper && !rBumpLast) {
                skib0 = skib0 + 0.01;
                skib1 = skib1 - 0.01;
                startTime = System.currentTimeMillis();
            }
            rBumpLast = gamepad1.right_bumper;

            if (gamepad1.left_bumper && !lBumpLast) {
                skib0 = skib0 - 0.01;
                skib1 = skib1 + 0.01;
            }
            lBumpLast = gamepad1.left_bumper;

            angleTurret0.setPosition(skib0);
            //angleTurret1.setPosition(skib1);
            
            telemetry.addLine("" + skib0);
            telemetry.addLine("" + skib1);
            telemetry.update();

           /* if (gamepad1.a && !aLast) {
                aPressable = !aPressable;
            }
            if (aPressable) {
                turret.setPower(-1);
            }
            else {
                turret.setPower(0);
            }
            aLast = gamepad1.a;*/
            //telemetry.addData("Turret Power", turret.getCurrentPosition()/elapsedTime);
            telemetry.update();
        }
    }
}