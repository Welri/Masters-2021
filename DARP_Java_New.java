import darp_pack.*;
import java.io.*;
import java.util.*;

public class DARP_Java_New{
    public int rows, cols, MaxIter, dcells,i,j;
    public boolean imp;
    public int[][] EnvironmentGrid,A;
    public double CCvariation;
    public double randomLevel;
    public static void main(String[] args){
        DARP_Java_New t = new DARP_Java_New();
        // t.rows = 6;
        // t.cols = 6;
        // t.MaxIter = 1000;
        // t.dcells = 30;
        // t.CCvariation = 0.001;
        // t.randomLevel = 0.0001;
        // t.imp = false;
        // t.EnvironmentGrid = new int[][]{{0,0,0,2,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,2,0,0},{0,0,0,0,0,0}};
     
        // DARP problem = new DARP(t.rows, t.cols, t.EnvironmentGrid, t.MaxIter, t.CCvariation, t.randomLevel, t.dcells, t.imp);
        // problem.constructAssignmentM();
        // t.A = problem.getAssignmentMatrix();

        // Read Input
        try {
            File file_in = new File("Input.txt");
            // System.out.println(new File("DARP_JPype//Value.txt").getAbsolutePath());
            BufferedReader br = new BufferedReader(new FileReader(file_in));
            String st;
            if ((st = br.readLine()) != null) {
                t.rows = Integer.parseInt(st);
            }
            if ((st = br.readLine()) != null) {
                t.cols = Integer.parseInt(st);
            }
            if ((st = br.readLine()) != null) {
                t.MaxIter = Integer.parseInt(st);
            }
            if ((st = br.readLine()) != null) {
                t.dcells = Integer.parseInt(st);
            }
            if ((st = br.readLine()) != null) {
                t.CCvariation = Double.parseDouble(st);
            }
            if ((st = br.readLine()) != null) {
                t.randomLevel = Double.parseDouble(st);
            }

            t.EnvironmentGrid = new int[t.rows][t.cols];
            for (t.i = 0; t.i < t.rows; t.i++) {
                for (t.j = 0; t.j < t.cols; t.j++) {
                    if ((st = br.readLine()) != null) {
                        t.EnvironmentGrid[t.i][t.j] = Integer.parseInt(st);
                    }
                }
            }
            if ((st = br.readLine()) != null) {
                t.imp = Boolean.parseBoolean(st);
            }

            DARP problem = new DARP(t.rows, t.cols, t.EnvironmentGrid, t.MaxIter, t.CCvariation, t.randomLevel, t.dcells, t.imp);
            problem.constructAssignmentM();
            t.A = problem.getAssignmentMatrix();
            
            br.close();
            System.out.println("hi");
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

        // Write Output
        try {
            File file_out = new File("Output_A.txt");
            BufferedWriter brw = new BufferedWriter(new FileWriter(file_out));
            for (t.i = 0; t.i < t.rows; t.i++) {
                for (t.j = 0; t.j < t.cols; t.j++) {
                    brw.write(String.valueOf(t.A[t.i][t.j]));
                    brw.write("\n");
                }
            }
            brw.close();
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

    }
}