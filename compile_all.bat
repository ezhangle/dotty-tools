@echo off

call "%VS100COMNTOOLS%\vsvars32.bat"

cl.exe /DWINDOWS /c utilities.c

cl.exe /DWINDOWS bnpts2npts.c utilities.obj
cl.exe /DWINDOWS data_size.c utilities.obj

cl.exe /DWINDOWS gen_analytic.c utilities.obj
cl.exe /DWINDOWS noff2npts.c utilities.obj

cl.exe /DWINDOWS scale.c utilities.obj
cl.exe /DWINDOWS shift_data.c utilities.obj
cl.exe /DWINDOWS swap_yz.c utilities.obj
cl.exe /DWINDOWS ysort.c utilities.obj

pause
