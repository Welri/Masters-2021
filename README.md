Important File Breakdown
------------------------

* DARP_Python_Main.py 
    * This file sends things to Input.txt, runs the subprocess (which runs DARP_Java_Main.java) and then reads from the Output.txt files.
    * This is also where the main development happens after DARP_Java_Main.java is run. STC should be run from here (or any other individual area search algoirhthm).
* DARP_Java_New.java
    * Runs the Java DARP algorithm (uses Input.txt and writes to Output.txt files)
* RUN_Java.bat
    * Batch file: Compiles and Runs the Java DARP_Java_New.java for WINDOWS
* RUN_Java.sh
    * Bash file: Compiles and Runs the Java DARP_Java_New.java for UBUNTU

TODO
----

* Write Bash File so that the code can run on LINUX
