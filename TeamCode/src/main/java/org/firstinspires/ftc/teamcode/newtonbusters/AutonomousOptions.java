package org.firstinspires.ftc.teamcode.newtonbusters;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;



/**
 * Created by Aryoman on 2/16/2016.
 * revised by JASMINE on 2/16/2016.
 */

@TeleOp(name="Autonomous Options", group="Main")

public class AutonomousOptions extends OpMode {

    //enum State  {DisplayAll, DisplayDelay, DisplayEndPos}
    enum State  {DisplayAll, DisplaySingle}

    State menuState = State.DisplayAll;

    SharedPreferences prefs;
    SharedPreferences.Editor editor;

    //private static final Map<String, String[]> OPTIONS = createMap();
    //Iterator<Map.Entry<String, String[]>> it  = OPTIONS.entrySet().iterator();;

    int selectionIdx = 0;

    static final String DELAY_PREF = "delay";
    static final String END_POS_PREF = "endPos";
    static final String NONE = "none";

    static final String[] DELAYS = {"0 sec", "3 sec", "5 sec", "7 sec"};
    static final String[] END_POS = {"floor goal", "beacon zone"};

    public static SharedPreferences getSharedPrefs(HardwareMap hardwareMap) {
        return hardwareMap.appContext.getSharedPreferences("autonomous", 0);
    }


    private static Map<String, String[]> prefMap = new HashMap<String, String[]>();
    static {
        prefMap.put(DELAY_PREF, DELAYS);
        prefMap.put(END_POS_PREF, END_POS);
    }
    private static String[] prefKeys = prefMap.keySet().toArray(new String[prefMap.keySet().size()]);
    static {
        Arrays.sort(prefKeys);
    }
    private static int keyIdx = 0;


    public int getIndex(String val, String[] array) {
        if(array!=null){
            for (int i = 0; i < array.length; i++) {
                if (array[i].equals(val)) {
                    return i;
                }
            }
        }
        return -1;
    }

    @Override
    public void init() {
        prefs = getSharedPrefs(hardwareMap);

        editor = prefs.edit();
        for (String key : prefs.getAll().keySet()) {
            telemetry.addData(key, prefs.getString(key, NONE));
        }
    }


    @Override
    public void loop() {
        switch (menuState) {
            case DisplayAll:
                telemetry.addData(DELAY_PREF, prefs.getString(DELAY_PREF, NONE));
                telemetry.addData(END_POS_PREF,prefs.getString(END_POS_PREF,NONE));
                displayAll();
                break;
            case DisplaySingle:
                displaySingle();
                break;
        }
    }

    void displayAll () {

        telemetry.clear();
        telemetry.addData("Choose", "X - accept Y - change");
        for (String key : prefKeys) {
            telemetry.addData(key, prefs.getString(key, NONE));
        }
        if (gamepad1.y) {
            while (gamepad1.y) {
            }
            menuState = State.DisplaySingle;
        }
    }

    void displaySingle () {

        telemetry.clear();
        telemetry.addData("Choose", "X - accept Y - change");
        String key = prefKeys[keyIdx];
        String[] array = prefMap.get(key);
        if (key != null) {
            String prefValue = prefs.getString(key, NONE);
            selectionIdx = getIndex(prefValue, array);
            telemetry.addData(key, prefValue);
        }
        if (gamepad1.x) {
            while (gamepad1.x) {
            }
            int nextKeyIdx = keyIdx+1;
            if (nextKeyIdx >= prefKeys.length) {
                keyIdx = 0;
                menuState = State.DisplayAll;
            }
            else {
                keyIdx = nextKeyIdx;
            }
        }
        if (gamepad1.y) {
            while (gamepad1.y) {
            }
            selectionIdx++;
            if (selectionIdx >= array.length) {
                selectionIdx = 0;
            }
            editor.putString(key, array[selectionIdx]);
            editor.apply();
        }
    }

}

