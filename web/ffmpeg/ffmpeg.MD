# FFMPEG

1. FFMPEG网站中的FFMPEG分为三个版本，Static，Shared，Dev。

    Static版本里面只有3个应用程序，ffmpeg.exe，ffplay.exe，ffprobe.exe，每个exe的体积都很大，相关的Dll已经被编译到exe里面去了。

   Shared版本里面除了3个应用程序：ffmpeg.exe，ffplay.exe，ffprobe.exe之外，还有一些Dll，比如说avcodec-54.dll之类的。Shared里面的exe体积很小，他们在运行的时候，到相应的Dll中调用功能。

   Dev版本是用于开发的，里面包含了库文件xxx.lib以及头文件xxx.h，这个版本不包含exe文件。

2. ffmpeg.exe，ffplay.exe，ffprobe.exe 三个工具的功能

   ffmpeg.exe：用于转码的应用程序。

   ffplay.exe：用于播放的应用程序。

   ffprobe.exe: 查看文件的应用程序。

3.  获取视频相关格式

   ```
   // 获取视频格式相关的参数
   FFmpegProbeResult probeResult = ffprobe.probe("C:/ffmpeg/rename.mp4");
   FFmpegFormat format = probeResult.getFormat();
   System.out.format("%nFile: '%s' ; Format: '%s' ; Duration: %.3fs",
   	format.filename,
   	format.format_long_name,
   	format.duration
   );
   
   FFmpegStream stream = probeResult.getStreams().get(0);
   System.out.format("%nCodec: '%s' ; Width: %dpx ; Height: %dpx",
   	stream.codec_long_name,
   	stream.width,
   	stream.height
   );
   ```

4. 转换视频


