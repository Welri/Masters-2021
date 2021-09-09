import darp_pack.*;
import java.io.*;
import java.util.*;

/* USER NOTES ON DARP
    Enew = Eold * CriterionMatrix * RandoMatrix * ConnectivityMatrix

    Criterion matrix - restores fairness between robots
    Connectivity matrix - ensures connectedness amongst cells
    Random matrix - to resolve issues with sharing

*/

public class DARP_Java_New {
    public int rows, cols, MaxIter, dcells, i, j, discr_achieved, obs, iterations, nr, r;
    public boolean imp, success;
    public boolean[] connected_bool;
    public int[] ArrayOfElements;
    public int[][] EnvironmentGrid, A;
    public int[][][] Ilabel_final;
    public ArrayList<boolean[][]> robotBinaryRegions;
    public double CCvariation;
    public double randomLevel;
    public int distanceMeasure;

    public static void main(String[] args) {
        DARP_Java_New t = new DARP_Java_New();
        
        // Read input and run DARP
        try {
            File file_in = new File("Input.txt");
            // System.out.println(new File("DARP_JPype//Value.txt").getAbsolutePath());
            BufferedReader br = new BufferedReader(new FileReader(file_in));
            String st;
            if ((st = br.readLine()) != null) {
                t.nr = Integer.parseInt(st);
            }
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
            if ((st = br.readLine()) != null) {
                t.distanceMeasure = Integer.parseInt(st);
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
                // Does not work for "0" and "1" but it works for "true" and "false"
                t.imp = Boolean.parseBoolean(st);
            }

            DARP problem = new DARP(t.rows, t.cols, t.EnvironmentGrid, t.MaxIter, t.CCvariation, t.randomLevel,
                    t.dcells, t.imp, t.distanceMeasure);
            problem.constructAssignmentM();
            // Collecting outputs
            t.A = problem.getAssignmentMatrix();
            t.discr_achieved = problem.getAchievedDiscr();
            t.success = problem.getSuccess();
            t.robotBinaryRegions = problem.getBinrayRobotRegions();
            t.obs = problem.getNumOB();
            t.iterations = problem.getIterations();
            t.connected_bool = problem.getConnectedBool();
            t.Ilabel_final = problem.getIlabel();
            t.ArrayOfElements = problem.getArrayOfElements();
            br.close();
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

        // Write output
        try {
            File file_out = new File("Output.txt");
            BufferedWriter brw = new BufferedWriter(new FileWriter(file_out));
            for (t.i = 0; t.i < t.rows; t.i++) {
                for (t.j = 0; t.j < t.cols; t.j++) {
                    brw.write(String.valueOf(t.A[t.i][t.j]));
                    brw.write("\n");
                }
            }
            brw.write(String.valueOf(t.discr_achieved));
            brw.write("\n");
            brw.write(String.valueOf(t.success));
            brw.write("\n");
            brw.write(String.valueOf(t.obs));
            brw.write("\n");
            brw.write(String.valueOf(t.iterations));
            brw.write("\n");
            for (t.r = 0; t.r < t.nr; t.r++) {
                brw.write(String.valueOf(t.ArrayOfElements[t.r]));
                brw.write("\n");
            }
            for (t.r = 0; t.r < t.nr; t.r++) {
                brw.write(String.valueOf(t.connected_bool[t.r]));
                brw.write("\n");
            }
            for (t.r = 0; t.r < t.nr; t.r++) {
                for (t.i = 0; t.i < t.rows; t.i++) {
                    for (t.j = 0; t.j < t.cols; t.j++) {
                        brw.write(String.valueOf(t.Ilabel_final[t.r][t.i][t.j]));
                        brw.write("\n");
                    }
                }
            }
            brw.close();
            // System.out.println("AHHHHHH");
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

    }
}