package org.firstinspires.ftc.teamcode.pedro;

public class CoolMath {
    //Returns angle in Radians of (x2,y2) relative to (x1,y1)
    public static double coolArctan(double x1, double y1, double x2, double y2) {
        if (y1 - y2 < 0) {
            if (x1 - x2 < 0) {
                return Math.atan2(y1 - y2, x1 - x2);
            } else {
                return Math.PI + Math.atan2(y1 - y2, x1 - x2);
            }
        } else {
            if (x1 - x2 > 0) {
                return Math.PI + Math.atan2(y1 - y2, x1 - x2);
            } else {
                return 2*Math.PI + Math.atan2(y1 - y2, x1 - x2);
            }
        }
    }
}
