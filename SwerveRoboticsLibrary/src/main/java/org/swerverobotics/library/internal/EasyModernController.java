package org.swerverobotics.library.internal;

import android.util.Log;
import com.qualcomm.hardware.*;
import com.qualcomm.modernrobotics.*;
import com.qualcomm.robotcore.eventloop.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.exception.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.*;
import org.swerverobotics.library.*;
import java.util.concurrent.*;

/**
 * Common to both easy modern servo and motor controllers
 */
public abstract class EasyModernController extends ModernRoboticsUsbDevice implements IOpModeStateTransitionEvents
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public static final String LOGGING_TAG = SynchronousOpMode.LOGGING_TAG;

    protected OpMode                        context;
    protected EventLoopManager              eventLoopManager;
    protected boolean                       isArmed;
    protected String                        targetName;
    protected HardwareMap.DeviceMapping     targetDeviceMapping;
    protected final RobotUsbDevice          robotUsbDevice;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public EasyModernController(OpMode context, ModernRoboticsUsbDevice target, DummyReadWriteRunnableStandard readWriteRunnable) throws RobotCoreException, InterruptedException
        {
        // We *have to* give a live ReadWriteRunnable to our parent constructor, so we grudgingly do so
        super (target.getSerialNumber(), SwerveThreadContext.getEventLoopManager(), readWriteRunnable);

        // Wait until the thing actually gets going
        readWriteRunnable.semaphore.acquire();

        // Then shut it down right away, because we want to start disarmed until we fully configure
        closeModernRoboticsUsbDevice(this);

        // Initialize the rest of our state
        this.context          = context;
        this.eventLoopManager = SwerveThreadContext.getEventLoopManager();
        this.isArmed          = false;

        ReadWriteRunnableStandard readWriteRunnableStandard = MemberUtil.getReadWriteRunnableModernRoboticsUsbDevice(target);
        ReadWriteRunnableUsbHandler handler                 = MemberUtil.getHandlerOfReadWriteRunnableStandard(readWriteRunnableStandard);
        this.robotUsbDevice                                 = MemberUtil.getRobotUsbDeviceOfReadWriteRunnableUsbHandler(handler);
        }

    //----------------------------------------------------------------------------------------------
    // Arming and disarming
    //----------------------------------------------------------------------------------------------

    public boolean isArmed()
        {
        return this.isArmed;
        }

    public abstract void arm();

    public abstract void disarm();

    static void closeModernRoboticsUsbDevice(ModernRoboticsUsbDevice usbDevice)
        // Close down the usbDevice in a robust and reliable way
        {
        // Get access to the state
        ExecutorService service = MemberUtil.getExecutorServiceModernRoboticsUsbDevice(usbDevice);
        ReadWriteRunnableStandard readWriteRunnableStandard = MemberUtil.getReadWriteRunnableModernRoboticsUsbDevice(usbDevice);

        // Stop accepting new work
        service.shutdown();

        // Set a dummy handler so that we don't end up closing the actual FT_device
        RobotUsbDevice robotUsbDevice = new DummyRobotUsbDevice();
        ReadWriteRunnableUsbHandler dummyHandler = new ReadWriteRunnableUsbHandler(robotUsbDevice);
        MemberUtil.setHandlerOfReadWriteRunnableStandard(readWriteRunnableStandard, dummyHandler);

        // Set 'running' to false; this fixes a race condition
        MemberUtil.setRunningReadWriteRunnableStandard(readWriteRunnableStandard, false);

        // Ok: actually carry out the close
        readWriteRunnableStandard.close();

        // Wait until the thread terminates
        Util.awaitTermination(service);
        }

    void installReadWriteRunnable(ModernRoboticsUsbDevice usbDevice, int cbMonitor, int ibStart)
        {
        try
            {
            ExecutorService service = Executors.newSingleThreadScheduledExecutor();
            ReadWriteRunnableStandard rwRunnable = new DummyReadWriteRunnableStandard(usbDevice.getSerialNumber(), this.robotUsbDevice, cbMonitor, ibStart, false);
            //
            MemberUtil.setExecutorServiceModernRoboticsUsbDevice(usbDevice, service);
            MemberUtil.setReadWriteRunnableModernRoboticsUsbDevice(usbDevice, rwRunnable);
            service.execute(rwRunnable);
            rwRunnable.blockUntilReady();
            rwRunnable.setCallback(usbDevice);
            this.eventLoopManager.registerSyncdDevice(rwRunnable);
            }
        catch (Exception e)
            {
            Util.handleCapturedException(e);
            }
        }

    //----------------------------------------------------------------------------------------------
    // HardwareDevice
    //----------------------------------------------------------------------------------------------

    public abstract String getDeviceName();

    //----------------------------------------------------------------------------------------------
    // Shims
    //----------------------------------------------------------------------------------------------

    /**
     * This class implements a dummy RobotUsbDevice that will apparently successfully do reads and
     * writes but doesn't actually do anything.
     */
    static class DummyRobotUsbDevice implements RobotUsbDevice
        {
        byte cbExpected = 0;
        @Override public void close()  {}
        @Override public void setBaudRate(int i) throws RobotCoreException {}
        @Override public void setDataCharacteristics(byte b, byte b1, byte b2) throws RobotCoreException  {}
        @Override public void setLatencyTimer(int i) throws RobotCoreException {}
        @Override public void purge(Channel channel) throws RobotCoreException {}
        @Override public int read(byte[] bytes) throws RobotCoreException { return this.read(bytes, bytes.length, 0/*bogus*/); }
        @Override public void write(byte[] bytes) throws RobotCoreException
            {
            // Write commands have zero-sized responses, read commands indicate their expected size
            byte bCommand = bytes[2];
            this.cbExpected = bCommand==0 ? 0/*write*/ : bytes[4] /*read*/;
            }
        @Override public int read(byte[] bytes, int cbReadExpected, int timeout) throws RobotCoreException
            {
            // Need to set the 'sync' bytes correctly, and set the sizes
            bytes[0]    = (byte)0x33;
            bytes[1]    = (byte)0xCC;
            bytes[4]    = (byte)cbExpected;
            return cbReadExpected;
            }
        }

    /**
     * This class is a ReadWriteRunnableStandard but one that doesn't report any errors
     * due to connection failures in its blockUntilReady()
     */
    static class DummyReadWriteRunnableStandard extends ReadWriteRunnableStandard
        {
        Semaphore semaphore = new Semaphore(0);

        public DummyReadWriteRunnableStandard(SerialNumber serialNumber, RobotUsbDevice device, int monitorLength, int startAddress, boolean debug)
            {
            super(serialNumber, device, monitorLength, startAddress, debug);
            }

        @Override public void run()
            {
            Thread.currentThread().setName("DummyReadWriteRunnableStandard.run");
            this.semaphore.release();
            super.run();
            }

        @Override public void blockUntilReady() throws RobotCoreException, InterruptedException
            {
            // Do nothing. In particular, don't report any errors
            }
        }

    //----------------------------------------------------------------------------------------------
    // IOpModeStateTransitionEvents
    //----------------------------------------------------------------------------------------------

    public abstract void stopHardware();

    @Override synchronized public boolean onUserOpModeStop()
        {
        Log.d(LOGGING_TAG, String.format("EasyModern: auto-stopping %s...", this.getSerialNumber().toString()));
        if (this.isArmed())
            {
            this.stopHardware();  // mirror StopRobotOpMode
            this.disarm();
            }
        Log.d(LOGGING_TAG, "EasyModern: ... done");
        return true;    // unregister us
        }

    @Override synchronized public boolean onRobotShutdown()
        {
        Log.d(LOGGING_TAG, String.format("EasyModern: auto-closing %s...", this.getSerialNumber().toString()));

        // We actually shouldn't be here by now, having received a onUserOpModeStop()
        // after which we should have been unregistered. But we close down anyway.
        this.close();

        Log.d(LOGGING_TAG, "EasyModern: ... done");
        return true;    // unregister us
        }
    }
