package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.google.gson.Gson;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.*;
import java.lang.reflect.Type;

/**
 * FileEditor is used for a pretty simple purpose: the reading and writing of files in the Android
 * environment. It's an abstract class that contains a number of useful static methods - no need to
 * construct or inherit from anything.
 *
 * @author Jack Thompson, Bubbert Innovations #18351
 */

public abstract class FileEditor  {

    public static <T> void writeSer(T object, Context context, String filePath,
                                        Telemetry errorLog) {
        try {
            File location = new File(context.getExternalFilesDir(null) + "/" + filePath);
            ObjectOutputStream outputStream = new ObjectOutputStream(new FileOutputStream(location));

            outputStream.writeObject(object);
            outputStream.close();
        }
        catch (IOException e) {
            if (errorLog != null) {
                errorLog.addData(e.toString(), e.getMessage());
                errorLog.update();
            }
            else {
                e.printStackTrace();
            }
        }
    }

    @SuppressWarnings("unchecked")
    public static <T> T readSer(Context context, String filePath, Telemetry errorLog) {
        try {
            File location = new File(context.getExternalFilesDir(null) + "/" + filePath);
            ObjectInputStream inputStream = new ObjectInputStream(new FileInputStream(location));

            return (T) inputStream.readObject();
        }
        catch (IOException | ClassNotFoundException e) {
            if (errorLog != null) {
                errorLog.addData(e.toString(), e.getMessage());
                errorLog.update();
            }
            else {
                e.printStackTrace();
            }
            return null;
        }
    }

    public static <T> void writeJSON(T object, Context context, Type inputType, String filePath, Telemetry errorLog)
    {
        try {
            Gson gson = new Gson();
            File location = new File(context.getExternalFilesDir(null) + "/" + filePath);
            PrintWriter out = new PrintWriter(new FileOutputStream(location));
            out.write(gson.toJson(object, inputType));

            out.close();
        }
        catch (IOException e) {
            if (errorLog != null) {
                errorLog.addData(e.toString(), e.getMessage());
                errorLog.update();
            }
            else {
                e.printStackTrace();
            }
        }
    }

    public static <T> T readJSON(Context context, Type outputType, String filePath, Telemetry errorLog) {
        try {
            Gson gson = new Gson();
            File location = new File(context.getExternalFilesDir(null) + "/" + filePath);
            System.out.println(location.getAbsolutePath());
            InputStreamReader reader = new InputStreamReader(new FileInputStream(location));

            return gson.fromJson(reader, outputType);
        }
        catch (IOException e) {
            if (errorLog != null) {
                errorLog.addData(e.toString(), e.getMessage());
                errorLog.update();
            }
            else {
                e.printStackTrace();
            }
            return null;
        }
    }

    /* **************************** BIG OLD LIST OF OVERLOADED METHODS *****************************
     * Basically, each overload simply offers you a choice: whether you want to include a Telemetry
     * errorLog, and whether you want to pass in the appContext, or pass in a HardwareMap instead to
     * retrieve it from. It's simply for user convenience.
    */
    public static <T> void writeJSON(T object, Context context, Type inputType, String filePath) {
        writeJSON(object, context, inputType, filePath, null);
    }

    public static <T> void writeJSON(T object, HardwareMap hwMap, Type inputType, String filePath) {
        writeJSON(object, hwMap.appContext, inputType, filePath, null);
    }

    public static <T> void writeJSON(T object, HardwareMap hwMap, Type inputType, String filePath, Telemetry errorLog) {
        writeJSON(object, hwMap.appContext, inputType, filePath, errorLog);
    }


    public static <T> T readJSON(Context context, Type outputType, String filePath) {
        return readJSON(context, outputType, filePath, null);
    }

    public static <T> T readJSON(HardwareMap hwMap, Type outputType, String filePath) {
        return readJSON(hwMap.appContext, outputType, filePath, null);
    }

    public static <T> T readJSON(HardwareMap hwMap, Type outputType, String filePath, Telemetry errorLog) {
        return readJSON(hwMap.appContext, outputType, filePath, errorLog);
    }


    public static <T> void writeSer(T object, Context context, String filePath) {
        writeSer(object, context, filePath, null);
    }

    public static <T> void writeSer(T object, HardwareMap hwMap, String filePath) {
        writeSer(object, hwMap.appContext, filePath, null);
    }

    public static <T> void writeSer(T object, HardwareMap hwMap, String filePath, Telemetry errorLog) {
        writeSer(object, hwMap.appContext, filePath, errorLog);
    }


    public static <T> T readSer(Context context, String filePath) {
        return readSer(context, filePath, null);
    }

    public static <T> T readSer(HardwareMap hwMap, String filePath) {
        return readSer(hwMap.appContext, filePath, null);
    }

    public static <T> T readSer(HardwareMap hwMap, String filePath, Telemetry errorLog) {
        return readSer(hwMap.appContext, filePath, errorLog);
    }



}
