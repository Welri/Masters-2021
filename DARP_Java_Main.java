// import java.time.chrono.MinguoChronology;

import darp_pack.*;
import java.io.*;
import java.util.*;

public class DARP_Java_Main {
    public int[][] grid, A;
    public int frows, fcols, fmaxiter, fdcells, i, j;
    public double fCC, fRL;
    public boolean fImp;

    public static void main(String[] args) {
        DARP_Java_Main t = new DARP_Java_Main();
        // Read Input
        try {
            File file_in = new File("Input.txt");
            // System.out.println(new File("DARP_JPype//Value.txt").getAbsolutePath());
            BufferedReader br = new BufferedReader(new FileReader(file_in));
            String st;
            if ((st = br.readLine()) != null) {
                t.frows = Integer.parseInt(st);
            }
            if ((st = br.readLine()) != null) {
                t.fcols = Integer.parseInt(st);
            }
            if ((st = br.readLine()) != null) {
                t.fmaxiter = Integer.parseInt(st);
            }
            if ((st = br.readLine()) != null) {
                t.fdcells = Integer.parseInt(st);
            }
            if ((st = br.readLine()) != null) {
                t.fCC = Double.parseDouble(st);
            }
            if ((st = br.readLine()) != null) {
                t.fRL = Double.parseDouble(st);
            }

            Main m = new Main();
            m.setRows(t.frows);
            m.setCols(t.fcols);
            m.setMaxIter(t.fmaxiter);
            m.setDcells(t.fdcells);
            m.setCC(t.fCC);
            m.setRL(t.fRL);
            t.grid = new int[t.frows][t.fcols];
            for (t.i = 0; t.i < t.frows; t.i++) {
                for (t.j = 0; t.j < t.fcols; t.j++) {
                    if ((st = br.readLine()) != null) {
                        t.grid[t.i][t.j] = Integer.parseInt(st);
                    }
                }
            }
            if ((st = br.readLine()) != null) {
                t.fImp = Boolean.parseBoolean(st);
            }
            m.setGird(t.grid);
            m.setImp(t.fImp);
            t.A = m.darp_func();
            br.close();
            // System.out.println("hi");
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
        // Write Output
        try {
            File file_out = new File("Output_A.txt");
            BufferedWriter brw = new BufferedWriter(new FileWriter(file_out));
            for (t.i = 0; t.i < t.frows; t.i++) {
                for (t.j = 0; t.j < t.fcols; t.j++) {
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
