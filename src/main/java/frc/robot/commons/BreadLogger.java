package frc.robot.commons;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;

// Logger class
public class BreadLogger {

    // Variables
    public static List<String> files = new ArrayList<>();
    public final String path;

    // Constructor
    public BreadLogger(String fileName) {
        this.path = Filesystem.getOperatingDirectory() + "/" + fileName + ".csv";
        files.add(fileName);
        clear();
    }

    // Method to clear this file
    public void clear() {
        try {
            new FileWriter(path).close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Method to write to this file
    public void write(String... content) {
        if (content.length > 0) {
            String toWrite = "";
            for (int i = 0; i < content.length - 1; i++) {
                toWrite = toWrite + content[i] + ", ";
            }
            toWrite = toWrite + content[content.length - 1] + "\n";
            try {
                FileWriter w = new FileWriter(path, true);
                w.write(toWrite);
                w.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } 
    }

    // Overload to write to this file
    public void write(int... content) {
        String[] stringContent = new String[content.length];
        for (int i = 0; i < content.length; i++) {
            stringContent[i] = String.valueOf(content[i]);
        }
        write(stringContent);
    }

    // Overload to write to this file
    public void write(double... content) {
        String[] stringContent = new String[content.length];
        for (int i = 0; i < content.length; i++) {
            stringContent[i] = String.valueOf(content[i]);
        }
        write(stringContent);
    }
    
}
