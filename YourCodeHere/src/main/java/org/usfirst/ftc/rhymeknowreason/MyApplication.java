package org.usfirst.ftc.rhymeknowreason;

import android.app.Application;

/**
 * Created by oeiplaptop2 on 3/9/16.
 */
public class MyApplication extends Application {
    private static MyApplication instance;

    public static MyApplication get(){
        return instance;
    }

    @Override
    public void onCreate(){
        super.onCreate();
        instance = this;
    }
}
