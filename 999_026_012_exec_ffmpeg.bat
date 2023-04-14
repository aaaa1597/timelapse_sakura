rem このパッチファイルを実行すると、ffmpegで連番静止画からmp4動画が生成される。
cd  D:\Products\000_PriDev\timelapse_sakura\ffmpeg\bin
D:
rem 10 frames per second
set fps=10
ffmpeg.exe -r %fps% -i ../../pictures\026_011_for_ffmpeg/%%03d.jpg ../../026_sakura_movie_ph1.mp4
