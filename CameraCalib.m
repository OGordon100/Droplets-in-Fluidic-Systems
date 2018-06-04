function [cameraParams, worldPoints] = CameraCalib(calibdir,num_imgs,...
    square_size,CameraStream)

%% Acquire New Images

% Capture images and store to file
disp('Move checkboard and press any key to capture:')
for loop_capture = 1:num_imgs
    disp(['Image ',num2str(loop_capture)])

    % Preview webcam input
    ImgData = cameraPreview(CameraStream);
    
    % Write image
    imwrite(ImgData,[calibdir,'\Calibration\Calib_',...
        num2str(loop_capture),'.png'])
end

% Stop camera
close(gcf)
clear('color_vid')

%% Calibrate

% Collate list of image names
calib_files = cell(1, num_imgs);
for loop_names = 1:num_imgs
    calib_files{loop_names} = fullfile([pwd,'\',calibdir,...
        '\Calibration\Calib_',num2str(loop_names),'.png']);
end

% Detect checkerboard points (had to make adjustments because of need for
% even/odd points)
[imagePoints, boardSize] = detectCheckerboardPoints(calib_files);
worldPoints = generateCheckerboardPoints(boardSize, square_size);

% Plot found checkerboard points
figure();
for loop_checker = 1:num_imgs
    I = imread(calib_files{loop_checker});
    subplot(2, 10, loop_checker);
    imshow(I);
    axis equal
    hold on;
    plot(imagePoints(:,1,loop_checker),...
        imagePoints(:,2,loop_checker),'ro');
end

cameraParams = estimateCameraParameters(imagePoints, worldPoints);

% Graph calibration accuracy.
figure();
showReprojectionErrors(cameraParams);
title('Reprojection Errors');

figure();
showExtrinsics(cameraParams);

% Save results if approved
if strcmp(questdlg('Approve Calibration?'),'Yes') == 1
    save([calibdir,'\Calibration\cameraParams.mat'],'cameraParams',...
        'worldPoints')
else
    error('Error: Calibration Denied!')
end

close all

end