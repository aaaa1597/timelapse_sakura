rem ���̃p�b�`�t�@�C�������s����ƁAffmpeg�ŘA�ԐÎ~�悩��mp4���悪���������B
cd  D:\Products\000_PriDev\timelapse_sakura\ffmpeg\bin
D:
rem 10 frames per second
set fps=10
ffmpeg.exe -r %fps% -i ../../pictures\026_011_for_ffmpeg/%%03d.jpg ../../026_sakura_movie_ph1.mp4
