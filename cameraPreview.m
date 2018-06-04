function ImgData = cameraPreview(color_vid)

% Make preview axis
fig = figure('NumberTitle','off','MenuBar','none');
set(fig, 'DeleteFcn', @closePreviewWindow_Callback);
set(fig, 'KeyPressFcn', @closePreviewWindow_Callback);
fig.Position = [300 300 830 420];
fig.Name = 'Camera Previewer';
im = imshow(zeros([color_vid.height,color_vid.width]...
    ,'uint8'));
axis image
axis off

% Display data until close
capturing = 1;
while capturing == 1
    ImgData = color_vid.getsnapshot;
    set(im, 'CData', ImgData);
    drawnow;
end

    function closePreviewWindow_Callback(~, ~)
        % Stop previewing & close window
        capturing = 0;
        close(gcf)
        
        % Find HebiCam object and clear it
        %AllVars = evalin('base', 'whos');
        %HebiLogic = strcmp({AllVars.class}, 'HebiCam');
        %HebiVar = {AllVars(HebiLogic).name};
        %evalin('base', strcat('clear ''',char(HebiVar),''''))
    end
end
