/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller;

import android.app.ActionBar;
import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.pm.ApplicationInfo;
import android.content.res.Configuration;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.IBinder;
import android.net.wifi.p2p.WifiP2pDevice;
import android.os.*;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.Gravity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.ftccommon.FtcRobotControllerService.FtcRobotControllerBinder;
import com.qualcomm.ftccommon.LaunchActivityConstantsList;
import com.qualcomm.ftccommon.Restarter;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.ftcrobotcontroller.opmodes.FtcOpModeRegister;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.robocol.*;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.Dimmer;
import com.qualcomm.robotcore.util.ImmersiveMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.wifi.WifiDirectAssistant;

import static junit.framework.Assert.*;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.Serializable;
import java.text.SimpleDateFormat;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.regex.Pattern;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

import org.swerverobotics.library.*;
import org.swerverobotics.library.internal.*;

public class FtcRobotControllerActivity extends Activity {

  public static final String TANK_BOT_CONFIG = "tankBot";
  public static final String K9_ARM_CONFIG = "k9bot arm";

  private static final int REQUEST_CONFIG_WIFI_CHANNEL = 1;
  private static final boolean USE_DEVICE_EMULATION = false;
  private static final int NUM_GAMEPADS = 2;

  public static final String CONFIGURE_FILENAME = "CONFIGURE_FILENAME";

  protected WifiManager.WifiLock wifiLock;
  protected SharedPreferences preferences;

  protected UpdateUI.Callback callback;
  protected Context context;
  private Utility utility;
  protected ImageButton buttonMenu;

  protected TextView textDeviceName;
  protected TextView textWifiDirectStatus;
  protected TextView textRobotStatus;
  protected TextView textWifiDirectPassphrase;
  protected TextView[] textGamepad = new TextView[NUM_GAMEPADS];
  protected TextView textOpMode;
  protected TextView textBuildDate;
  protected TextView textErrorMessage;
  protected ImmersiveMode immersion;

  protected SwerveUpdateUIHook updateUI;
  protected Dimmer dimmer;
  protected LinearLayout entireScreenLayout;

  protected FtcRobotControllerService controllerService;

  protected FtcEventLoop eventLoop;
  protected Queue<UsbDevice> receivedUsbAttachmentNotifications;

  protected class RobotRestarter implements Restarter {

    public void requestRestart() {
      requestRobotRestart();
    }

  }

  protected ServiceConnection connection = new ServiceConnection() {
    @Override
    public void onServiceConnected(ComponentName name, IBinder service) {
      FtcRobotControllerBinder binder = (FtcRobotControllerBinder) service;
      onServiceBind(binder.getService());
    }

    @Override
    public void onServiceDisconnected(ComponentName name) {
      controllerService = null;
    }
  };

  @Override
  protected void onNewIntent(Intent intent) {
    super.onNewIntent(intent);

    if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(intent.getAction())) {
      UsbDevice usbDevice = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
      if (usbDevice != null) {  // paranoia
        // We might get attachment notifications before the event loop is set up, so
        // we hold on to them and pass them along only when we're good and ready.
        if (receivedUsbAttachmentNotifications != null) { // *total* paranoia
          receivedUsbAttachmentNotifications.add(usbDevice);
          passReceivedUsbAttachmentsToEventLoop();
        }
      }
    }
  }

  protected void passReceivedUsbAttachmentsToEventLoop() {
    if (this.eventLoop != null) {
      for (;;) {
        UsbDevice usbDevice = receivedUsbAttachmentNotifications.poll();
        if (usbDevice == null)
          break;
        this.eventLoop.onUsbDeviceAttached(usbDevice);
      }
    }
    else {
      // Paranoia: we don't want the pending list to grow without bound when we don't
      // (yet) have an event loop
      while (receivedUsbAttachmentNotifications.size() > 100) {
        receivedUsbAttachmentNotifications.poll();
      }
    }
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    receivedUsbAttachmentNotifications = new ConcurrentLinkedQueue<UsbDevice>();
    eventLoop = null;

    setContentView(R.layout.activity_ftc_controller);

    utility = new Utility(this);
    context = this;
    entireScreenLayout = (LinearLayout) findViewById(R.id.entire_screen);
    buttonMenu = (ImageButton) findViewById(R.id.menu_buttons);
    buttonMenu.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View v) {
        openOptionsMenu();
      }
    });

    textDeviceName = (TextView) findViewById(R.id.textDeviceName);
    textWifiDirectStatus = (TextView) findViewById(R.id.textWifiDirectStatus);
    textRobotStatus = (TextView) findViewById(R.id.textRobotStatus);
    textOpMode = (TextView) findViewById(R.id.textOpMode);
    textBuildDate = (TextView) findViewById(R.id.textBuildDate);
    textErrorMessage = (TextView) findViewById(R.id.textErrorMessage);
    textWifiDirectPassphrase = (TextView) findViewById(R.id.textWifiDirectPassphrase);
    textGamepad[0] = (TextView) findViewById(R.id.textGamepad1);
    textGamepad[1] = (TextView) findViewById(R.id.textGamepad2);
    immersion = new ImmersiveMode(getWindow().getDecorView());
    dimmer = new Dimmer(this);
    dimmer.longBright();
    Restarter restarter = new RobotRestarter();

    updateUI = new SwerveUpdateUIHook(this, dimmer);
    updateUI.setRestarter(restarter);
    updateUI.setTextViews(textWifiDirectStatus, textRobotStatus,
            textGamepad, textOpMode, textErrorMessage, textDeviceName);
    callback = updateUI.new CallbackHook();

    PreferenceManager.setDefaultValues(this, R.xml.preferences, false);
    preferences = PreferenceManager.getDefaultSharedPreferences(this);

    WifiManager wifiManager = (WifiManager) getSystemService(Context.WIFI_SERVICE);
    wifiLock = wifiManager.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "");

    hittingMenuButtonBrightensScreen();

    if (USE_DEVICE_EMULATION) { HardwareFactory.enableDeviceEmulation(); }

    try {
      ApplicationInfo ai = getPackageManager().getApplicationInfo(getPackageName(), 0);
      ZipFile zf = new ZipFile(ai.sourceDir);
      ZipEntry ze = zf.getEntry("classes.dex");
      long time = ze.getTime();
      String buildDate = SimpleDateFormat.getInstance().format(new java.util.Date(time));
      zf.close();
      textBuildDate.setText("App updated on " + buildDate);
    } catch(Exception e) {}
  }

  @Override
  protected void onStart() {
    super.onStart();

    // save 4MB of logcat to the SD card
    RobotLog.writeLogcatToDisk(this, 4 * 1024);

    Intent intent = new Intent(this, FtcRobotControllerService.class);
    bindService(intent, connection, Context.BIND_AUTO_CREATE);

    utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);

    callback.wifiDirectUpdate(WifiDirectAssistant.Event.DISCONNECTED);

    entireScreenLayout.setOnTouchListener(new View.OnTouchListener() {
    @Override
    public boolean onTouch(View v, MotionEvent event) {
      dimmer.handleDimTimer();
      return false;
      }
    });

    wifiLock.acquire();
  }

  @Override
  protected void onResume() {
    super.onResume();
  }

  @Override
  public void onPause() {
    super.onPause();
  }

  @Override
  protected void onStop() {
    super.onStop();

    if (controllerService != null) unbindService(connection);

    RobotLog.cancelWriteLogcatToDisk(this);

    wifiLock.release();
  }

  @Override
  public void onWindowFocusChanged(boolean hasFocus){
    super.onWindowFocusChanged(hasFocus);
    // When the window loses focus (e.g., the action overflow is shown),
    // cancel any pending hide action. When the window gains focus,
    // hide the system UI.
    if (hasFocus) {
      if (ImmersiveMode.apiOver19()){
        // Immersive flag only works on API 19 and above.
        immersion.hideSystemUI();
      }
    } else {
      immersion.cancelSystemUIHide();
    }
  }


  @Override
  public boolean onCreateOptionsMenu(Menu menu) {
    getMenuInflater().inflate(R.menu.ftc_robot_controller, menu);
    return true;
  }

  @Override
  public boolean onOptionsItemSelected(MenuItem item) {
    int id = item.getItemId();
    if (id==R.id.action_restart_robot) {
        dimmer.handleDimTimer();
        Toast.makeText(context, "Restarting Robot", Toast.LENGTH_SHORT).show();
        requestRobotRestart();
        return true;
    }
    if (id==R.id.action_settings) {
      // The string to launch this activity must match what's in AndroidManifest of FtcCommon for this activity.
      Intent settingsIntent = new Intent("com.qualcomm.ftccommon.FtcRobotControllerSettingsActivity.intent.action.Launch");
      startActivityForResult(settingsIntent, LaunchActivityConstantsList.FTC_ROBOT_CONTROLLER_ACTIVITY_CONFIGURE_ROBOT);
      return true;
    }
    if (id == R.id.action_manual_control) {
        final String filename = utility.getFilenameFromPrefs(R.string.pref_hardware_config_filename, Utility.NO_FILE);
        // gets name of the configuration file
        if(filename.equals(K9_ARM_CONFIG)) {
          Intent manualIntent = new Intent(this,K9ArmManualActivity.class);
          startActivity(manualIntent);
        } else {
          Toast.makeText(this,"No activity found for " + filename + ".",Toast.LENGTH_LONG).show();
        }
        return true;
    }
    if (id==R.id.action_about) {
        // The string to launch this activity must match what's in AndroidManifest of FtcCommon for this activity.
        Intent intent = new Intent("com.qualcomm.ftccommon.configuration.AboutActivity.intent.action.Launch");
        startActivity(intent);
        return true;
    }
    if (id==R.id.action_exit_app) {
        finish();
        return true;
    }
    if (id==R.id.action_view_logs) {
        // The string to launch this activity must match what's in AndroidManifest of FtcCommon for this activity.
        Intent viewLogsIntent = new Intent("com.qualcomm.ftccommon.ViewLogsActivity.intent.action.Launch");
        viewLogsIntent.putExtra(LaunchActivityConstantsList.VIEW_LOGS_ACTIVITY_FILENAME, RobotLog.getLogFilename(this));
        startActivity(viewLogsIntent);
        return true;
    }
        return super.onOptionsItemSelected(item);
    }

  @Override
  public void onConfigurationChanged(Configuration newConfig) {
    super.onConfigurationChanged(newConfig);
    // don't destroy assets on screen rotation
  }

  @Override
  protected void onActivityResult(int request, int result, Intent intent) {
    if (request == REQUEST_CONFIG_WIFI_CHANNEL) {
      if (result == RESULT_OK) {
        Toast toast = Toast.makeText(context, "Configuration Complete", Toast.LENGTH_LONG);
        toast.setGravity(Gravity.CENTER, 0, 0);
        showToast(toast);
      }
    }
    if (request == LaunchActivityConstantsList.FTC_ROBOT_CONTROLLER_ACTIVITY_CONFIGURE_ROBOT) {
      if (result == RESULT_OK) {
        Serializable extra = intent.getSerializableExtra(FtcRobotControllerActivity.CONFIGURE_FILENAME);
        if (extra != null) {
          utility.saveToPreferences(extra.toString(), R.string.pref_hardware_config_filename);
          utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
        }
      }
    }
  }

  public void onServiceBind(FtcRobotControllerService service) {
    DbgLog.msg("Bound to Ftc Controller Service");
    controllerService = service;
    updateUI.setControllerService(controllerService);

    callback.wifiDirectUpdate(controllerService.getWifiDirectStatus());
    callback.robotUpdate(controllerService.getRobotStatus());
    requestRobotSetup();
  }

  private void requestRobotSetup() {
    if (controllerService == null) return;

    FileInputStream fis = fileSetup();
    // if we can't find the file, don't try and build the robot.
    if (fis == null) { return; }

    HardwareFactory factory;

    // Modern Robotics Factory for use with Modern Robotics hardware
    HardwareFactory modernRoboticsFactory = new HardwareFactory(context);
    modernRoboticsFactory.setXmlInputStream(fis);
    factory = modernRoboticsFactory;

    eventLoop = new SwerveFtcEventLoop(factory, new FtcOpModeRegister(), callback, this);

    controllerService.setCallback(callback);
    controllerService.setupRobot(eventLoop);

    passReceivedUsbAttachmentsToEventLoop();
  }

  private FileInputStream fileSetup() {

    final String filename = Utility.CONFIG_FILES_DIR
        + utility.getFilenameFromPrefs(R.string.pref_hardware_config_filename, Utility.NO_FILE) + Utility.FILE_EXT;

    FileInputStream fis;
    try {
      fis = new FileInputStream(filename);
    } catch (FileNotFoundException e) {
      String msg = "Cannot open robot configuration file - " + filename;
      utility.complainToast(msg, context);
      DbgLog.msg(msg);
      utility.saveToPreferences(Utility.NO_FILE, R.string.pref_hardware_config_filename);
      fis = null;
    }
    utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
    return fis;
  }

  private void requestRobotShutdown() {
    if (controllerService == null) return;
    controllerService.shutdownRobot();
  }

  private void requestRobotRestart() {
    requestRobotShutdown();
    requestRobotSetup();
  }

  protected void hittingMenuButtonBrightensScreen() {
    ActionBar actionBar = getActionBar();
    if (actionBar != null) {
      actionBar.addOnMenuVisibilityListener(new ActionBar.OnMenuVisibilityListener() {
        @Override
        public void onMenuVisibilityChanged(boolean isVisible) {
          if (isVisible) {
            dimmer.handleDimTimer();
          }
        }
      });
    }
  }

  public void showToast(final Toast toast) {
    runOnUiThread(new Runnable() {
      @Override
      public void run() {
        toast.show();
      }
    });
  }

    //==============================================================================================

    public static String LOGGING_TAG = SynchronousOpMode.LOGGING_TAG;

    /**
     * We are being notified that the FTC robot controller activity is being shut down.
     *
     * In response, we choose here to also terminate the underlying process. While not normally
     * something one does in a well-behaved and fully-debugged Android app, we do it here in order
     * to make the controller behavior more robust and reliable: there are known scenarios in which
     * the controller can get into an inoperable state that *requires* that this underlying process
     * be terminated (that is, there's something in that process state that's stuck). While this can
     * be done manually with 'swiping' closed the process, relying on the user to do that is just
     * silly. By terminating the process here, that will happen automatically when the user chooses
     * 'Exit' from the menu.
     *
     * @see <a href="http://developer.android.com/reference/android/app/Activity.html">Activity Life Cycle</a>
     */
    @Override protected void onDestroy()
        {
        // Do required superclass stuff
        super.onDestroy();

        // Commit suicide
        Log.i(LOGGING_TAG, "FtcRobotControllerActivity committing process suicide");
        int pid = android.os.Process.myPid();
        android.os.Process.killProcess(pid);
        }


    //==============================================================================================
    // Hooking infrastructure (Swerve)
    //
    // The code below has been added to the stock FtcRobotControllerActivity in order to hook
    // into state transitions of various kinds that happen within the robot controller application.
    // Most of what's here is of necessity pretty obscure and technical in nature, but
    // fortunately those details won't be of significance to most.

    static class SwerveEventLoopMonitorHook implements EventLoopManager.EventLoopMonitor
    // Hook to receive event monitor state transition
        {
        //------------------------------------------------------------------------------------------
        // State
        //------------------------------------------------------------------------------------------

        // The previously installed hook
        final EventLoopManager.EventLoopMonitor prevMonitor;

        // The activity in which we live
        final FtcRobotControllerActivity activity;

        //------------------------------------------------------------------------------------------
        // Construction
        //------------------------------------------------------------------------------------------

        SwerveEventLoopMonitorHook(EventLoopManager.EventLoopMonitor prevMonitor, FtcRobotControllerActivity activity)
            {
            this.prevMonitor = prevMonitor;
            this.activity    = activity;
            }

        // Make sure we're installed in the in the hook of the current event loop
        public synchronized static void installIfNecessary(FtcRobotControllerService service, FtcRobotControllerActivity activity)
            {
            if (service == null)
                return;

            Robot robot = MemberUtil.robotOfFtcRobotControllerService(service);
            if (robot == null)
                return;

            EventLoopManager eventLoopManager = robot.eventLoopManager;
            if (eventLoopManager == null)
                return;

            // Ok, the EventLoopManager is up and running. Install our hooks if we haven't already done so

            EventLoopManager.EventLoopMonitor monitor = eventLoopManager.getMonitor();
            if (monitor != null)
                {
                if (monitor instanceof SwerveEventLoopMonitorHook)
                    {
                    // we're already installed
                    }
                else
                    {
                    SwerveEventLoopMonitorHook newMonitor = new SwerveEventLoopMonitorHook(monitor, activity);
                    eventLoopManager.setMonitor(newMonitor);
                    Log.v(SynchronousOpMode.LOGGING_TAG, "installed SwerveEventLoopMonitorHook");
                    }
                }
            }

        //------------------------------------------------------------------------------------------
        // Notifications
        //------------------------------------------------------------------------------------------

        @Override
        public void onStateChange(RobotState newState)
            {
            this.prevMonitor.onStateChange(newState);
            RobotStateTransitionNotifier.onRobotStateChange(newState);
            }

        @Override
        public void onErrorOrWarning()
          {
          this.prevMonitor.onErrorOrWarning();
          }
        }

    class SwerveUpdateUIHook extends UpdateUI
    // Hook used to augment the user interface
        {
        //------------------------------------------------------------------------------------------
        // State
        //------------------------------------------------------------------------------------------

        FtcRobotControllerActivity activity;
        FtcRobotControllerService  controllerService;

        //------------------------------------------------------------------------------------------
        // Construction
        //------------------------------------------------------------------------------------------

        SwerveUpdateUIHook(FtcRobotControllerActivity activity, Dimmer dimmer)
            {
            super(activity, dimmer);
            this.activity = activity;
            this.controllerService = null;
            }

        @Override
        public void setControllerService(FtcRobotControllerService controllerService)
            {
            super.setControllerService(controllerService);
            this.controllerService = controllerService;
            }

        //------------------------------------------------------------------------------------------
        // Operations
        //------------------------------------------------------------------------------------------

        class CallbackHook extends UpdateUI.Callback
            {
            //--------------------------------------------------------------------------------------
            // Operations
            //--------------------------------------------------------------------------------------

            @Override
            public void robotUpdate(final String status)
            // Called from FtcRobotControllerService.reportRobotStatus(). That is called from many
            // places, but in particular it is called *immediately* after RobotFactory.createRobot()
            // is called in FtcRobotControllerService.run(); that ensures we get to see the raw
            // initial state.
                {
                // Make sure we get to see all the robot state transitions
                SwerveEventLoopMonitorHook.installIfNecessary(controllerService, FtcRobotControllerActivity.this);

                super.robotUpdate(status);
                RobotStateTransitionNotifier.onRobotUpdate(status);
                }

            @Override
            public void wifiDirectUpdate(WifiDirectAssistant.Event event)
                {
                super.wifiDirectUpdate(event);

                final String message = controllerService == null
                        ? ""
                        : String.format("Wifi Direct passphrase: %s", controllerService.getWifiDirectAssistant().getPassphrase());

                SwerveUpdateUIHook.this.activity.runOnUiThread(new Runnable()
                {
                @Override
                public void run()
                    {
                    activity.textWifiDirectPassphrase.setText(message);
                    }
    });
  }

            }
        }
}




















