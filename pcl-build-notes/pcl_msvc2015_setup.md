# Building PCL on Windows 10 using MSVC 2015

These instructions include how to install, setup, and run PCL on Windows 10 using pre-built binaries.  This guide will assume one is using Microsoft's Visual Studio 2015 on a 64-bit machine.  Note: this guide is a essentially paraphrasing the linked document.

### Step 0: Install PCL   
go to the provided link and install the all-in-one installer for your verson of windows and msvc. [link here](http://unanancyowen.com/?p=1255&lang=en).  Install PCL at the recommended location.

### Step 1: Set environment variables   
go to control panel->system->advanced system settings
click on the Environment Variables button and enter the following information
```
VARIABLE       VALUE
---------------------------------------------
PCL_ROOT       C:\Program Files\PCL 1.7.2
Path           ;%PCL_ROOT%\bin
               ;%PCL_ROOT%\3rdParty\FLANN
               ;%PCL_ROOT%\3rdParty\VTK\bin
```  


