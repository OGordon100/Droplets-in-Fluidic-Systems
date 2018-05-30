function [cameraParams, worldPoints] = getCalib(calibdir,calib_style,...
    num_imgs,square_size,CameraStream)

if calib_style == 'get_new'
    % Get new calibration
    PassCalib = 0;
    while PassCalib == 0
        try
            [cameraParams, worldPoints] = CameraCalib(calibdir,num_imgs,...
                square_size,CameraStream);
            PassCalib = 1;
        catch
            disp('Camera Calibration Failed!')
        end
    end
    
else
    % Load previous calibration
    if exist([calibdir,'\Calibration\cameraParams.mat'],'file') == 0
        error('Error. No calibration file found!')
    end
    load([calibdir,'\Calibration\cameraParams.mat']);
    
end

end