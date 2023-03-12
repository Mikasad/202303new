md c:\temp
copy %~dp0\*.dll     c:\temp\
copy %~dp0\*.dll     c:\windows\system32\
copy %~dp0\*.ocx     c:\temp\
copy %~dp0\*.ocx     c:\windows\system32\
for %%i in (c:\temp\*.ocx) do  regsvr32 /s %%i
for %%i in (c:\temp\*.dll) do  regsvr32 /s %%i