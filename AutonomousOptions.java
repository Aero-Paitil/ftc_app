package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Aryoman on 2/16/2016.
 * revised by JASMINE on 2/16/2016.
 */
public class AutonomousOptions extends OpMode {

    enum State  {DisplayAll, DisplayDelay, DisplayEndPos}

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

    /*private static Map<String, String[]> createMap() {
        Map<String, String[]> result = new HashMap<String, String[]>();
       result.put(DELAY_PREF, DELAYS);
        result.put(END_POS_PREF, END_POS);

        return Collections.unmodifiableMap(result);
    }*/

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

    private void selectOption(String key, String[] array, State state) {
        if (key != null) {
            String prefValue = prefs.getString(key, NONE);
            selectionIdx = getIndex(prefValue, array);
            telemetry.addData(key, prefValue);
        }
        telemetry.addData("Choose", "X - accept Y - change");
        if (gamepad1.x) {
            while (gamepad1.x) {
                telemetry.addData("Success", "");
            }
            menuState = state;
            telemetry.clearData();
        }
        if (gamepad1.y) {
            while (gamepad1.y) {
            }
            if(key!=null) {
                selectionIdx++;
                if (selectionIdx >= array.length) {
                    selectionIdx = 0;
                }
                editor.putString(key, array[selectionIdx]);
                editor.apply();
            }else{
                menuState=state;
                telemetry.clearData();
            }
        }
    }

    @Override
    public void loop() {
        switch (menuState) {
            case DisplayAll:
                telemetry.addData(DELAY_PREF, prefs.getString(DELAY_PREF, NONE));
                telemetry.addData(END_POS_PREF,prefs.getString(END_POS_PREF,NONE));
                selectOption(null, null, State.DisplayDelay);
                break;
            case DisplayDelay:
                selectOption(DELAY_PREF, DELAYS, State.DisplayEndPos);
                break;
            case DisplayEndPos:
                selectOption(END_POS_PREF, END_POS, State.DisplayAll);
                break;
        }
    }
}




/*        switch (menuState) {
            case DisplayAll:
                telemetry.addData(ALLIANCE_PREF, prefs.getString(ALLIANCE_PREF, NONE));
                telemetry.addData(DELAY_PREF, prefs.getString(DELAY_PREF, NONE));
                telemetry.addData(END_POS_PREF, prefs.getString(END_POS_PREF, NONE));
                telemetry.addData("Choose", "X - accept Y - change");
                if (gamepad1.x) {
                    while (gamepad1.x) {
                        telemetry.addData("Success", "");
                    }
                }
                if (gamepad1.y) {
                    while (gamepad1.y) {
                    }
                    menuState = MenuState.DisplayAlliance;
                    telemetry.clearData();
        }
                break;
            case DisplayAlliance:
                prefValue = prefs.getString(ALLIANCE_PREF, NONE);
                selectionIdx = getIndex(prefValue, ALLIANCE);
                telemetry.addData(ALLIANCE_PREF, prefValue);
                telemetry.addData("Choose", "X - accept Y - change");
                if (gamepad1.x) {
                    while (gamepad1.x) {
                    }
                    menuState = MenuState.DisplayDelay;
                    telemetry.clearData();
                } else if (gamepad1.y) {
                    while (gamepad1.y) {
                    }
                    selectionIdx++;

                    if (selectionIdx >= ALLIANCE.length) {
                        selectionIdx = 0;
                    }
                    editor.putString(ALLIANCE_PREF, ALLIANCE[selectionIdx]);
                    editor.apply();
                }
                break;
            case DisplayDelay:
                prefValue = prefs.getString(DELAY_PREF, NONE);
                telemetry.addData(DELAY_PREF, prefValue);
                selectionIdx = getIndex(prefValue, DELAYS);
                telemetry.addData("Choose", "X - accept Y - change");
                if (gamepad1.x) {
                    while (gamepad1.x) {
                    }

                    menuState = MenuState.DisplayEndPos;
                    telemetry.clearData();
                } else if (gamepad1.y) {
                    while (gamepad1.y) {
                    }
                    selectionIdx++;

                    if (selectionIdx >= DELAYS.length) {
                        selectionIdx = 0;

                    }
                    editor.putString(DELAY_PREF, DELAYS[selectionIdx]);
                    editor.apply();
                }
                break;
            case DisplayEndPos:
                prefValue = prefs.getString(END_POS_PREF, NONE);
                telemetry.addData(END_POS_PREF, prefValue);
                selectionIdx = getIndex(prefValue, END_POS);
                telemetry.addData("Choose", "X - accept Y - change");
                if (gamepad1.x) {
                    while (gamepad1.x) {
                    }

                    menuState = MenuState.DisplayAll;
                    telemetry.clearData();
                } else if (gamepad1.y) {
                    while (gamepad1.y) {
                    }
                    selectionIdx++;

                    if (selectionIdx >= END_POS.length) {
                        selectionIdx = 0;

                    }
                    editor.putString(END_POS_PREF, END_POS[selectionIdx]);
                    editor.apply();
                }
                break;

        }*/
