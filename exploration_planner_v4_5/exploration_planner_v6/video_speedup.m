inputVideo = VideoReader('result_map_1.mp4');

outputVideo = VideoWriter('result_map_1_fast.mp4', 'MPEG-4');
outputVideo.FrameRate = inputVideo.FrameRate;  % Keep original frame rate
open(outputVideo);

frameCount = 0;
while hasFrame(inputVideo)
    frameCount = frameCount + 1;
    frame = readFrame(inputVideo);
    if mod(frameCount, 10) == 1  % Keep every 10th frame
        writeVideo(outputVideo, frame);
    end
end

close(outputVideo);
