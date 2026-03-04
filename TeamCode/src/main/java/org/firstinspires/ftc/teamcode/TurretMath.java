package org.firstinspires.ftc.teamcode;

public class TurretMath {

    public static double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public static double angleToPoint(
            double robotX, double robotY,
            double targetX, double targetY) {

        return Math.atan2(targetY - robotY,
                targetX - robotX);
    }
}