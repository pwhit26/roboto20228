package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SpindexerPositions extends LinearOpMode {
    DcMotorEx spindexer;
    int pos=0;
    private boolean rBumpLast, lBumpLast;

    @Override
    public void runOpMode() throws InterruptedException {
        spindexer=hardwareMap.get(DcMotorEx.class, "spindexer");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setPower(0.3);

        waitForStart();
        while (opModeIsActive())
        {
            if (gamepad1.right_bumper && !rBumpLast) {
                pos = pos + 10;
            }
            rBumpLast = gamepad1.right_bumper;

            if (gamepad1.left_bumper && !lBumpLast) {
                pos = pos - 10;
            }
            lBumpLast = gamepad1.left_bumper;

            spindexer.setTargetPosition(pos);
            //angleTurret1.setPosition(skib1);

            telemetry.addLine("" + pos);
            telemetry.update();



        }

    }
}
