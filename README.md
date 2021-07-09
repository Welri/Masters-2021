Important File Breakdown
------------------------

General
-------
* DARP_Python_Main.py 
    * Where main development happens - NOT JUST DARP RELATED (needs a name change)
    * Compiles and runs java files using subprocesses (.sh and .bat)
    * Runs both DARP and PrimMST algorithm
* compiling .bat/.sh
    * Batch file / Bash file: WINDOWS / UBUNTU
    * Compiles all the necessary Java files
    * Gets called in DARP_Python_Main.py
* Logging_xxx.txt
    * Log files that store information about algorithm runs
    * Written to from DARP_Pythin_Main.py

DARP Related
-------------
* DARP_data
    * Code used to check cases where DARP broke historically - a lot of the debugging happened here
    * Target_case_checker is easier to use. You just have to follow a specific file format.
* darp_pack
    * Contains code copied from original developers of DARP algorith,
* DARP_Java_New.java
    * Runs the Java DARP algorithm (uses Input.txt and writes to Output.txt files)
    * DARP related files it interfaces with are in darp_pack
* RUN_Java .bat/.sh
    * Batch / Bash file:  WINDOWS / UBUNTU
    * Runs the Java DARP_Java_New.java
* Input.txt / Output.txt
    * Files that are written to and from for DARP

MST Related - NOT CURRENTLY USED
------------
The code to interface with this is still in DARP_Python_Main.py, it is simply commented out (note the indent).
These areas are labelled "JAVA MST COMMENTED OUT".
Wrote an alternative in python under DARP_Python_Main.py. 
It slows the code down - in 1km^2 area it now consumed 95% of computation time instead of 80%.
However it is easier to interface - Going to do development there and just eventually migrate it all to Java - potentially only going to happen post hand-in of project.

* STC_pack
    * prim.java - coded adapted from codde by Aakash Hasija (used)
    * kruskal.java - code copied from Aakash Hasija (not used)
* MST_Input.txt / MST_Output.txt
    * Files that are written to and from for Prim's MST Algorithm
* Running_pMST.java
    * Contatins Code that writes to and from Input and Output files
    * Contains code to interface with STC_Pack (prim.java)
* pMST_Run_Java .bat/.sh
    * Runs Prim Java file (Running_pMST)
    * Gets called in DARP_Python_Main.py

   
    

