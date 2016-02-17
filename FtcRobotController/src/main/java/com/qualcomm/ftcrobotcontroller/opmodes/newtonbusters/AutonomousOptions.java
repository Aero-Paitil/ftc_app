package com.qualcomm.ftcrobotcontroller.opmodes.newtonbusters;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by Aryoman on 2/16/2016.
 */
public class AutonomousOptions extends OpMode {
    enum MenuState {DisplayAll, DisplayDelay, DisplayEndPos, UpdateDelay, UpdateEndPos}

    MenuState menuState = MenuState.DisplayAll;
    SharedPreferences prefs;
    SharedPreferences.Editor editor;

    
    int selectionIdx = 0;
    
    static String DELAY_PREF = "delay";
    static String END_POS_PREF = "endPos";
    
    static String [] DELAYS = {"0 sec", "3 sec", "5 sec", "7 sec"};
    static String [] END_POS = {"floor goal", "beacon zone"};    

    private static final Map<String, String[]> OPTIONS = createMap();

    private static Map<String, String[]> createMap() {
        Map<String, String[]> result = new HashMap<String, String[]>();
        result.put("alliance", new String[]{"red", "blue"});
        result.put(DELAY_PREF, new String[]{"0 sec", "3 sec", "5 sec", "7 sec"});
        result.put(END_POS_PREF, new String[]{"floor goal", "beacon zone"});

        return Collections.unmodifiableMap(result);
    }

    @Override
    public void init() {
        prefs = hardwareMap.appContext.getSharedPreferences("autonomous", 0);

        editor = prefs.edit();

        for (String key : prefs.getAll().keySet()) {
            telemetry.addData(key, prefs.getString(key, "none"));
        }
    }

    @Override
    public void loop() {
        switch (menuState) {
            case DisplayAll:
                telemetry.addData(DELAY_PREF, prefs.getString(DELAY_PREF, "none"));
                telemetry.addData(END_POS_PREF, prefs.getString(END_POS_PREF, "none"));
                telemetry.addData("Choose", "X - accept Y - change");
                if (gamepad1.x)
                {
                    while (gamepad1.x) {}
                    telemetry.addData("Options", "Using options selected");
                }
                if (gamepad1.y)
                {
                    while (gamepad1.y) {}
                    menuState = MenuState.DisplayDelay;
                    telemetry.clearData();
                    
                    selectionIdx = 0;
                }
                break; 
            case DisplayDelay:
                telemetry.addData(DELAY_PREF, prefs.getString(DELAY_PREF, DELAYS[0]));
                telemetry.addData("Choose", "X - accept Y - change");
                if (gamepad1.x){
                    while(gamepad1.x) {}
                    
                    menuState = MenuState.DisplayEndPos;
                    telemetry.clearData();
                }
                else if(gamepad1.y)
                {
                    while (gamepad1.y){}
                    selectionIdx++;
                    
                    if (selectionIdx >= DELAYS.length){
                        selectionIdx =0;
                        
                    }
                    editor.putString(DELAY_PREF, DELAYS[selectionIdx]);
                    editor.apply();
                }
                break;
            case DisplayEndPos:
                telemetry.addData(END_POS_PREF, prefs.getString(END_POS_PREF, END_POS[0]));
                telemetry.addData("Choose", "X - accept Y - change");
                if (gamepad1.x){
                    while(gamepad1.x) {}

                    menuState = MenuState.DisplayAll;
                    telemetry.clearData();
                }
                else if(gamepad1.y)
                {
                    while (gamepad1.y){}
                    selectionIdx++;

                    if (selectionIdx >= END_POS.length){
                        selectionIdx =0;

                    }
                    editor.putString(END_POS_PREF, END_POS[selectionIdx]);
                    editor.apply();
                }
                break;
        }

        
    }
}
