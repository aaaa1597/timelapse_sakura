@echo off
rem �Ԉ����̃o�b�`
cd D:\Products\000_PriDev\timelapse_sakura\pictures\026_006_Thinout
D:

if exist ..\..\026_007_CheckMakerfileResult.txt del ..\..\026_007_CheckMakerfileResult.txt

for %%f in (*.jpg) do (
  if exist ../026_008_makers/%%~nf.png (
    echo ../026_008_makers/%%~nf.png
  ) ^
  else (
    echo ../026_008_makers/%%~nf.png�͂Ȃ��B>> ../../026_007_CheckMakerfileResult.txt
  )
)
