function generateVideo(folder,frameName,frameFormat,nFrames,videoName)

videoFileName = fullfile(folder,videoName);
mov = avifile(videoFileName,'compression','None');

try
    for i=1:nFrames
        fName = sprintf('%s%.4d',frameName,i);
        str= fullfile(folder,fName);
        img = imread(str,frameFormat);
        F = im2frame(img);
        mov = addframe(mov,F);
    end
    mov = close(mov);
catch
    mov = close(mov);
    disp('Problem generating the movie.');
    return;
end
