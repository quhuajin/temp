:: Name
::    get_version.bat
::
:: COPYRIGHT
::    Copyright (c) 2009 Mako Surgical Corp.
::
:: SYNTAX
::    get_version.bat WCPATH >version.h
::
:: DESCRIPTION
::    This scripts generates version string.
::
:: SEE ALSO
:: 
:: $Revision: 4086 $
:: $Date: 2012-10-01 18:08:45 -0400 (Mon, 01 Oct 2012) $
:: $Author: rzhou $


@echo off
set "line=$$HeadURL: svn+ssh://svn.makosurgical.com/var/svnroot/repos/robot/Embedded/trunk/MiscSoftware/mCutter/ccs/get_version.bat $$"

call set "line=%%line:*/Embedded/=%%"

for /f "tokens=1 delims=:" %%a in ('cygpath -u %1') do set wp=%%a 

for /f "tokens=1 delims=:" %%a in ('svnversion %wp%') do set rev=%%a 

for /f "tokens=1,2,3 delims=/" %%a in ("%line%") do set d1=%%a&set d2=%%b

if "%d1%" == "tags" goto TAGFOUND
echo.const char g_usFirmwareVersion[20] = "%rev%";
goto END
:TAGFOUND
echo.const char g_usFirmwareVersion[20] = "%d2%";
:END