package org.usfirst.ftc.rhymeknowreason;

import android.content.Context;

import org.swerverobotics.library.SwerveUtil;
import org.swerverobotics.library.interfaces.*;
import org.swerverobotics.library.examples.*;

/**
 * MyRobotControllerAdministration is a container for 'administrative' methods that interact
 * with the Swerve Library. You don't <em>have to</em> put your administrative methods in a separate
 * class as we do here, but it does help keep them neat and tidy. Administrative methods are
 * each tagged with a Java annotation that connotes and bestows their significance; see the
 * individual example methods below for details. Note that administrative methods don't reside in any
 * given OpMode, but rather are used and invoked outside of the OpMode life cycle. Neither the
 * name of the administrative methods nor the name of the class are of significance; the only
 * important thing is the annotations with which they are decorated.
 *
 * As we've written things here, the code resides in an Android module named 'YourCodeHere' which
 * contains a package named 'org.usfirst.ftc.exampleteam.yourcodehere'. If you wish to change that,
 * or to create multiple such modules with different names, that's pretty straightforward: you can
 * begin by copying the entire YourCodeHere folder, then renaming the src\main\java\org\...\yourcodehere
 * path to reflect your new package name, whatever that might be. That new package name will also
 * of course need to appear in your copied .java files (including package-info.java), so those will
 * need their 'package' statement at the top adjusted accordingly. Perhaps less obviously,
 * the src\main\AndroidManifest.xml, which also contains the package name, will need to have the
 * 'package' attribute of the 'manifest' element adjusted. Finally, the new modules should be added
 * to the 'settings.gradle' file so they show up in Android Studio.
 *
 * @see TeleOp
 * @see Autonomous
 * @see OpModeRegistrar
 * @see OnRobotRunning
 * @see SynchTeleOp
 */
public class MyRobotControllerAdministration
    {

    @OnRobotRunning
    public static void playSoundOnRobotRunning(Context context)
        {
        SwerveUtil.playSound(context, R.raw.nxtstartup);
        }

    /**
     * Any public static method annotated with {@link OnRobotStartupFailure} is invoked when the robot
     * object in the robot controller application fails to enter the running state during
     * an attempt to do so. A common cause of such failures is a mismatch between the robot
     * configuration file and the devices currently attached to the robot.
     *
     * @param context   the application context of the robot controller application. Useful for
     *                  interacting with other parts of the Android system, such creating a
     *                  MediaPlayer.
     * @see #playSoundOnRobotRunning(Context)
     */
    @OnRobotStartupFailure
    public static void playSoundOnRobotStartupFailure(Context context)
        {
        SwerveUtil.playSound(context, R.raw.chord);
        }

    }
