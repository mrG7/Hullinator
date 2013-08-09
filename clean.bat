erase *.sdf
erase *.opensdf

rd .\ipch /s /q

REM remove the build folders (has compiled .exe files in it)
rd .\Debug /s /q
rd .\Release /s /q

rd .\Hullinator\Debug /s /q
rd .\Hullinator\Release /s /q

REM Remove the x64 build folder
rd .\x64 /s /q
rd .\Hullinator\x64 /s /q
