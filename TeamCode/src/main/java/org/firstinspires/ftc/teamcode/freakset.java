package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class freakset extends LinearOpMode {
    private DcMotorEx motor;
    int motorPos=0;
    private boolean rBumpLast, lBumpLast;
    @Override
    public void runOpMode() throws InterruptedException {
        rBumpLast=false;
        lBumpLast=false;
       motor=hardwareMap.get(DcMotorEx.class, "spindexer");

       motor.setTargetPosition(0);
       motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

       waitForStart();

       while(opModeIsActive())
       {

           if (gamepad1.right_bumper && !rBumpLast)
           {
               motorPos=motorPos+100;
           }
           rBumpLast = gamepad1.right_bumper;
           if (gamepad1.left_bumper && !lBumpLast)
           {
               motorPos=motorPos-100;
           }
           lBumpLast = gamepad1.left_bumper;
           motor.setTargetPosition(motorPos);
           telemetry.addData("", motorPos);
           telemetry.addData("", motor.getCurrentPosition());
           telemetry.update();

       }


    }
}
