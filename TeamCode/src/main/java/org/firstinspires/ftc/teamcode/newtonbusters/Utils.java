package org.firstinspires.ftc.teamcode.newtonbusters;

import com.qualcomm.ftccommon.DbgLog;

/**
 * Created by Aryoman on 12/2/2015.
 */
public class Utils {

    public static void delay(double seconds)
    {
        try {
            Thread.sleep((long) (seconds*1000));
        }
        catch (InterruptedException e) {
          DbgLog.error(e.getMessage());
        }
    }
}
