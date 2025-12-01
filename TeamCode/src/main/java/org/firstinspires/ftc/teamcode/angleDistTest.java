package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class angleDistTest extends LinearOpMode {

    public Servo angleTurret0, angleTurret1;
    public DcMotorEx turret;
    public Limelight3A limelight;
    double cameraMountPitchDeg = 30;
    double tagHeightM = 0.8;
    double cameraHeightM = 0.25;
    double power;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret=hardwareMap.get(DcMotorEx.class, "turret");
        angleTurret0=hardwareMap.get(Servo.class, "angleTurret0");
        angleTurret0.setPosition(0.13);
        angleTurret1=hardwareMap.get(Servo.class, "angleTurret1");
        angleTurret1.setPosition(0.87);
        limelight.pipelineSwitch(1);
        limelight.start();


        waitForStart();
        while(opModeIsActive())
        {

        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            double txDeg = 0.0;
            double tyDeg = 0.0;
            double ta = 0.0;
            boolean llValid = false;
            if (ll != null) {
                txDeg = ll.getTx();
                tyDeg = ll.getTy();
                ta = ll.getTa();
                llValid = ll.isValid();
            }

            double thetaV = Math.toRadians(cameraMountPitchDeg + tyDeg);
            if (Math.abs(Math.cos(thetaV)) > 1e-3) {
                double forwardZ = (tagHeightM - cameraHeightM) / Math.tan(thetaV);
                double thetaH = Math.toRadians(txDeg);
                double lateralX = forwardZ * Math.tan(thetaH);
                double verticalY = (tagHeightM - cameraHeightM);
                double euclid = Math.sqrt(lateralX * lateralX + verticalY * verticalY + forwardZ * forwardZ);
                if (forwardZ<2)
                {
                    forwardZ = forwardZ - 0.295;
                    euclid=euclid-0.295;
                    power = forwardZ;
                }
                else if (forwardZ>=2)
                {
                    forwardZ = forwardZ - 0.42;
                    euclid = euclid-0.42;
                }
                telemetry.addData("LL forwardZ (m)", forwardZ);
                telemetry.addData("LL distance (m)", euclid);
                telemetry.addData("LL valid", llValid);
                telemetry.addData("LL tx (deg)", txDeg);
                telemetry.addData("LL ty (deg)", tyDeg);
                telemetry.update();
            } else {
                telemetry.addData("LL forwardZ (m)", "undefined angle");
                telemetry.update();
            }
            telemetry.update();

        }
        if (gamepad1.x) {

            if (power<=1) {
                angleTurret0.setPosition(0);
                angleTurret1.setPosition(0.99);
                turret.setPower(-1);
            }
            else if (power>1) {
                //MAGICAL ANGLES
                angleTurret0.setPosition(0.04);
                angleTurret1.setPosition(0.95);
                turret.setPower(-1);

            }
        }


        if (gamepad1.y)
        {
            turret.setPower(0);
        }

        }
    }}
