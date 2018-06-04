% Oliver Gordon & Dominic Coy 4th Year Project
% Automatically control entire experiment!

clearvars -except PumpObj
close all
daqreset

%% Options

% Camera & Video
CameraRes = '1920x1080';                        % Camera resolution
CameraName = 'C920';                            % Camera name

% Calibration
CalibSquareSize = 2.0;                          % Square size (mm)
CalibImgs = 20;                                 % Number of calib images

% Object tracking
CompLogBoost = 6;                               % Drop tracker gain
CompThreshBrightness = 170;                     % Pixel brightness thresh
CompThreshNum = 275;                             % No. pixels at brightness

% DAQ
DAQRate = 3000;                                 % Sample rate
DAQChannelPD = 0;                               % Photodiode channel no.
DAQChannelMotor = 0;                            % Motor channel no.

% Pump
PumpDir = 1;                                    % 1 = dispense, 0 = suck
PumpDwell = 0.2;                                % Dwell time (?)
PumpVol = 0.02;                                % Volume (ml)
PumpFlowrate = 1.5;                             % Rate (ml/hr), max 60

% Motor
MotorDriveVolt = 1.6;                           % FET gate voltage
MotorBoostVolt = 1.6;                           % FET boost voltage

% Timing
MotorPauseTime = 2;                             % Image capture pause
MotorBoostTime = 0;                             % Motor boost
PDRecordTime = 35;                              % Total PD recording

% General
SaveRaw = 1;                                    % 1 = yes, 0 = no
CaptNewData = questdlg('Capture New Data?');    % 1 = yes, 0 = no
ExpDesc = inputdlg('Experiment Description:');  % Experiment description

% Droplet
Density = 1000;                                 % Density (kg m^-3)


%% Equipment Checks

% Ensure motor won't set on fire
if MotorDriveVolt > 5 || MotorBoostVolt > 8
    error('Error: Motor will fry!')
end

if MotorDriveVolt < 1.5
    error(['Error: Motor won''','t start!'])
end

% Ensure pump won't go on forever
if PumpFlowrate >=50 && PumpVol >=2
    error('Error: Pump will flow forever!')
end

% Engage camera
CameraStream = HebiCam(CameraName,'Resolution',CameraRes,'ImageMode',...
    'Gray');

%% Code Setup

% Set up appropriate file structure
if strcmp(CaptNewData,'Yes') == 1
    if exist(['Experiments\',date],'dir') == 0
        % If no experiment today
        activdir = ['Experiments\',date,'\1'];
    else
        % If already an experiment today
        allfiles = struct2cell(dir(['Experiments\',date]));
        allfiles_max = max(str2double(allfiles(1,3:end)));
        activdir = ['Experiments\',date,'\',...
            num2str(allfiles_max+1)];
    end
    mkdir(activdir);
    mkdir([activdir,'\Calibration'])
    mkdir([activdir,'\Data'])
    WorkNew = 1;
    
    filePh = fopen([activdir,'\Description.txt'],'w');
    fprintf(filePh,'%s\n',ExpDesc{:});
    fclose(filePh);
else
    % Pick old experiment
    activdir = uigetdir('Working Directory',...
        'Select Experiment Folder:');
    activdir = activdir(length(pwd)+2:end);
    WorkNew = 0;
end

% Acquire/Select calibration
if WorkNew == 1
    if strcmp(questdlg('Get New Calibration?'),'Yes') == 1
        % New data, new calibration
        [cameraParams, worldPoints] = getCalib(activdir,'get_new',...
            CalibImgs,CalibSquareSize,CameraStream);
    else
        % New data, old calibration
        calibdir = uigetdir('Working Directory',...
            'Select Experiment Folder:');
        copyfile([calibdir,'\Calibration\cameraParams.mat'],[activdir,...
            '\Calibration'])
        [cameraParams, worldPoints] = getCalib(calibdir,'get_old');
    end
else
    % Old data, old calibration
    [cameraParams, worldPoints] = getCalib(activdir,'get_old');
end

%% Equipment Setup

% Engage DAQ
DAQ = daq.createSession('ni');
DAQ.Rate = DAQRate;

addAnalogOutputChannel(DAQ,'Dev1', DAQChannelMotor, 'Voltage');
addAnalogInputChannel(DAQ,'Dev1', DAQChannelPD, 'Voltage');

% Engage pump

try
    if exist('PumpObj','var') == 1
        % Close COM session to prevent having to restart MATLAB
        fclose(PumpObj);
        clear('PumpObj')
    end
    PumpObj = turnpump(PumpDir, PumpDwell, PumpVol, PumpFlowrate);
catch
    fclose(instrfind);
    PumpObj = turnpump(PumpDir, PumpDwell, PumpVol, PumpFlowrate);
end

% Determine time pump will run for
PumpMaxRunTime = 50;%(PumpVol/PumpFlowrate*60)+10;

% Prepare voltages to output
MotorSignalWhileCollecting = ...
    [MotorBoostVolt.*ones(1,MotorBoostTime*DAQRate),...
    MotorDriveVolt.*ones(1,(PDRecordTime-MotorBoostTime)*DAQRate)];

%% Object Detection Setup

PassObjectSetup = 0;
while PassObjectSetup == 0
    
    % Take preview image & determine resolution resizing scale
    fprintf('Ensure experiment is in frame, and \npress any key...\n')
    ImgSetup = cameraPreview(CameraStream);
    
    % Identify key features on camera & determine scaled search ranges
    figure()
    imshow(ImgSetup)
    disp('Draw shape around region for...')
    
    % Image Recognition location
    disp('   Image Detection')
    MaskSize = uint8(createMask(impoly()));
    [BoundSize_y,BoundSize_x] = find(MaskSize);
    BoundRangeSize = [min(BoundSize_y),max(BoundSize_y)...
        ;min(BoundSize_x),max(BoundSize_x)];
    BoundCornersSize = [BoundRangeSize(1,1),BoundRangeSize(2,2);...
        BoundRangeSize(1,1),BoundRangeSize(2,1);...
        BoundRangeSize(1,2),BoundRangeSize(2,1);...
        BoundRangeSize(1,2),BoundRangeSize(2,2)];
    
    close(gcf)
    
    % Checkerboard Location
    [ImgSetupNoWarp, newOrigin] = undistortImage(...
        ImgSetup, cameraParams);
    [imagePoints_disp, boardSize_disp]=detectCheckerboardPoints(ImgSetup);
    try
        [imagePointsDrop, boardSize]=detectCheckerboardPoints(ImgSetupNoWarp);
    catch
        continue
    end
    
    BoundRangeCheck = [min(imagePoints_disp(:,2))-...
        diff(imagePoints_disp(1:2,2))...
        ,max(imagePoints_disp(:,2))+...
        diff(imagePoints_disp(end-1:end,2)); ...
        min(imagePoints_disp(:,1))-diff(imagePoints_disp(1:2,2))...
        ,max(imagePoints_disp(:,1))+diff(imagePoints_disp(end-1:end,2))];
    
    % Pull together all detection boundaries
    BoundCornersAll = [BoundRangeSize(2,1),BoundRangeSize(1,1),...
        diff(BoundRangeSize(2,:)),diff(BoundRangeSize(1,:)) ; ...
        BoundRangeCheck(2,1),BoundRangeCheck(1,1),...
        diff(BoundRangeCheck(2,:)),diff(BoundRangeCheck(1,:))];
    BoundRangesAll = BoundCornersSize;
    TBounds_x = BoundRangesAll(:,1);
    TBounds_y = BoundRangesAll(:,2);
    
    % Display image setup results
    im_fused = imfuse(ImgSetup,ImgSetupNoWarp);
    FigSetupFuse = figure();
    FigSetupFuse.Visible = 'off';
    imshow(im_fused)
    viscircles(imagePointsDrop,10.*ones(length(imagePointsDrop),1),...
        'LineWidth',0.5);
    
    FigSetup = figure();
    FigSetup.Visible = 'off';
    ImgSetupAnnotated = insertObjectAnnotation(ImgSetup,...
        'rectangle',...
        BoundCornersAll,{'Image Area','Checkerboard'},...
        'Color',{'yellow','red'},'LineWidth',3,'FontSize',30);
    imshow(ImgSetupAnnotated)
    viscircles(imagePoints_disp,10.*ones(length(imagePoints_disp),1),...
        'LineWidth',0.5);
    tilefigs([FigSetupFuse,FigSetup])
    FigSetupFuse.Visible = 'on';
    FigSetup.Visible = 'on';
    
    % Approve setup, and repeat capture if failing
    if strcmp(questdlg('Approve this camera setup?'),'Yes')==1
        PassObjectSetup = 1;
    end
    close(FigSetupFuse, FigSetup)
end

% Calculate extrinsics from calibration file & checkerboard
[R, t] = extrinsics(imagePoints_disp, worldPoints, cameraParams);

% Engage motor (if undervolted, step down to it)
if MotorDriveVolt < 4
    DAQ.outputSingleScan(4.5)
    pause(0.5)
    for SteppingV = 4:-0.2:MotorDriveVolt
        DAQ.outputSingleScan(SteppingV);
        pause(0.1)
    end
else
    DAQ.outputSingleScan(MotorDriveVolt);
end

% Take full carrier fluid screenshot and extract windowed regions
ImgSetupDetect = CameraStream.getsnapshot;
ImgDectImage = ImgSetupDetect(min(TBounds_x(1:4)):max(TBounds_x(1:4)),...
    min(TBounds_y(1:4)):max(TBounds_y(1:4)),:);

%% Pre-Allocation

% Make figure windows for displaying
FigImages = figure();

FigPreview = subplot(1,2,1);
img = imshow(ImgDectImage);
title('Snapshot Detector')
FigDropSize = subplot(1,2,2);
img2 = imshow(ImgDectImage);
AxImg = gca;
title('Snapshot Image')

FigPD = figure();

FigSpectra = subplot(1,2,1);
AxSpectra = gca;
xlabel('Time (s)','HandleVisibility','off')
ylabel('PD Intensity (V)','HandleVisibility','off')
set(gca,'NextPlot','replacechildren') ;
ylim([0 10])
FigFT = subplot(1,2,2);
AxFT = gca;
set(gca,'NextPlot','replacechildren') ;
xlabel('Frequency (Hz)')
ylabel('PSD (V^2 Hz^{1})')

tilefigs([FigImages,FigPD]);

% Set up loop and pre-calculate as much as possible
RadAll = NaN.*ones(1,500);
TicTocAll = NaN.*ones(1,(PumpMaxRunTime*30));
DNum = 1;
DNum1 = 0;
PumpRunLoop = 1;
PumpRunTime = 0;

LogBoost = log(CompLogBoost);

ImageRangex = min(TBounds_x(1:4)):max(TBounds_x(1:4));
ImageRangey = min(TBounds_y(1:4)):max(TBounds_y(1:4));

Log10 = 10*log(10);

RadiiAll = NaN.*ones(1,100);
FAll_n1 = NaN.*ones(1,100);
DeltaFAll_n1 = NaN.*ones(1,100);
FAll_n2 = NaN.*ones(1,100);
DeltaFAll_n2 = NaN.*ones(1,100);

keepgoing = 1;

%% Collect Data

% Start experiment!
disp(['Ready to run for ',num2str(PumpMaxRunTime),'s. Press any key!'])
pause();

% Start turning pump
fprintf(PumpObj,'RUN');

% Run until pump finished
while keepgoing == 1
    % Take video screenshot and extract windowed regions
    tic
    ImgDetect = CameraStream.getsnapshot();
    TicTocAll(PumpRunLoop) = toc;
    ImgImage = ImgDetect(ImageRangex,ImageRangey);
    
    % Determine drop location with pure carrier fluid screenshot
    CompImage = (ImgImage-ImgDectImage).*5.*LogBoost;
    %disp(sum(sum(CompImage>CompThreshBrightness)))
    if sum(sum(CompImage>CompThreshBrightness)) > CompThreshNum
        %% Hardware Control
        
        disp('Droplet in image location!')
        set(img, 'CData', CompImage);
        drawnow;
        
        % Turn off motor
        DAQ.outputSingleScan(0);
        pause(MotorPauseTime)
        
        % Take high res screenshot
        SizeImage = CameraStream.getsnapshot();
        ImgSize = SizeImage(min(TBounds_x(1:4)):max(TBounds_x(1:4)),...
            min(TBounds_y(1:4)):max(TBounds_y(1:4)),:);
        set(img2, 'CData', ImgSize);
        drawnow;
        
        % Turn on motor
        if MotorDriveVolt < 4
            DAQ.outputSingleScan(4.5)
            pause(0.1)
            for SteppingV = 4:-0.2:MotorDriveVolt
                DAQ.outputSingleScan(SteppingV);
                pause(0.05)
            end
        else
            DAQ.outputSingleScan(MotorDriveVolt);
        end
        
        % Prepare output
        queueOutputData(DAQ,MotorSignalWhileCollecting');
        [PDDataAll,TimeAll] = DAQ.startForeground;
        
        %% PD Analysis
        %axes(AxSpectra);
        
        % Find where the droplet goes through and remove mean
        PDIndex = findchangepts(PDDataAll,'MaxNumChanges',4);
        PDData = PDDataAll;
        Time = TimeAll;
        PDData = PDDataAll(PDIndex(1)-0:PDIndex(2)+00);%-mean(PDDataAll);
        Time = TimeAll(PDIndex(1)-0:PDIndex(2)+00);
        
        % Calculate PSD
        L = length(PDData);
        PDFT2 = abs(fft(PDData)/L);
        PDFT = PDFT2(1:floor(L/2+1));
        PDFT(2:end-1) = log(10).*2*PDFT(2:end-1);
        Freq = DAQRate*(0:(L/2))/L;
        
        % Find FWHM (n=1 and 2)
        [PeakPDFTAll_n1,PeakFreqAll_n1,FWHMAll_n1,PeakPromAll_n1] = ...
            findpeaks(PDFT(3:end),Freq(3:end),'SortStr','descend');
        PeakPDFT_n1 = PeakPDFTAll_n1(1);
        PeakFreq_n1 = PeakFreqAll_n1(1);   
        HM_n1 = PeakPDFTAll_n1(1)-PeakPromAll_n1(1)+(PeakPDFTAll_n1(1)/2);
        FWHM_n1 = FWHMAll_n1(1); 
        
        [PeakPDFTAll_n2,PeakFreqAll_n2,FWHMAll_n2,PeakPromAll_n2] = ...
            findpeaks(PDFT(100:end),Freq(100:end),'SortStr','descend');
        PeakPDFT_n2 = PeakPDFTAll_n2(1);
        PeakFreq_n2 = PeakFreqAll_n2(1);   
        HM_n2 = PeakPDFTAll_n2(1)-PeakPromAll_n2(1)+(PeakPDFTAll_n2(1)/2);
        FWHM_n2 = FWHMAll_n2(1); 
        
        % Commit results
        FAll_n1 = PeakFreq_n1;
        DeltaFAll_n1 = FWHM_n1;
        FAll_n2 = PeakFreq_n2;
        DeltaFAll_n2 = FWHM_n2;
        
        % Plot PD Signal
        subplot(1,2,1)
        plot(AxSpectra,TimeAll,PDDataAll)
        hold on
        subplot(1,2,1)
        plot(AxSpectra,Time,PDData,'r')
        hold off
        
        % Plot FT and FWHM
        subplot(1,2,2)
        plot(AxFT,Freq,PDFT)
        hold on;  
        plot(AxFT,PeakFreq_n1,PeakPDFT_n1,'rx')
        plot(AxFT,[PeakFreq_n1-FWHM_n1,PeakFreq_n1+FWHM_n1],[HM_n1,HM_n1],'r')
        plot(AxFT,[PeakFreq_n1-FWHM_n1/2,PeakFreq_n1+FWHM_n1/2],[HM_n1,HM_n1],'g')
        plot(AxFT,PeakFreq_n2,PeakPDFT_n2,'rx')
        plot(AxFT,[PeakFreq_n2-FWHM_n2,PeakFreq_n2+FWHM_n2],[HM_n2,HM_n2],'r')
        plot(AxFT,[PeakFreq_n2-FWHM_n2/2,PeakFreq_n2+FWHM_n2/2],[HM_n2,HM_n2],'g')
        xlim([0 35])
        ylim([0 3*max(PDFT(2:end))])
        hold off;
 
        %% Size Detection
        
        % Subtract foreground from background (auto determine bright/dark)
        ImNoBack = (ImgSize-ImgDectImage);
        %if max(max(ImNoBack)) < 100
        %    ImNoBack = (imcomplement(ImgSize)-imcomplement(ImgDectImage));
        %end
        
        % Binarise
        ImBin1 = ImNoBack;
        ImBin1(ImBin1<10)=0;
        ImBin1(ImBin1>20)=255;
        ImBin2 = imbinarize(ImBin1);
        
        % Make shape a convex hull and fill it in
        ImConvex = bwconvhull(ImBin2,'objects');
        
        % Find circle
        [CentersUnsrt, RadiiUnsrt] = imfindcircles(ImConvex,[10 25]);
        
        % Get rightmost droplet
        if isempty(CentersUnsrt) == 0
            [~,RightIndex] = max(CentersUnsrt(:,1));
            Centers = CentersUnsrt(RightIndex,:);
            Radii = RadiiUnsrt(RightIndex);
            BoundCorners = [Centers(1)-Radii,Centers(2)-Radii;...
                Centers(1)+Radii,Centers(2)-Radii];
        end
        
        % Plot
        viscircles(AxImg,CentersUnsrt, RadiiUnsrt,...
            'EdgeColor','r','LineWidth',0.1);
        
        % Remove windowing and account for distortion
        BoundCorners(:,1) = BoundCorners(:,1)+(min(TBounds_y(1:4)));
        BoundCorners(:,2) = BoundCorners(:,2)+(min(TBounds_x(1:4)));
        imagePointsDrop = undistortPoints(BoundCorners,cameraParams);
        
        % Compute the radius of the circle
        worldPointsDrop = pointsToWorld(cameraParams,R,t,imagePointsDrop);
        Diam = worldPointsDrop(2,:) - worldPointsDrop(1,:);
        RadiiAll(DNum) = hypot(Diam(1), Diam(2))/2;
        fprintf('Droplet radius = %0.2f mm\n', RadiiAll(DNum));
        
        % Commit result
        Radii = RadiiAll(DNum)*1e-3;
        
        Gamma_n2 = (4*Density*(Radii^2)/(pi*4))*(DeltaFAll_n1^2+FAll_n1^2);
        %Gamma_n3 = (4*Density*(Radii^2)/(pi*9))*(DeltaFAll_n2^2+FAll_n2^2)
        eta_n2 = pi*Density*DeltaFAll_n1*(Radii^2)/4;
        %eta_n3 = pi*Density*DeltaFAll_n2*(Radii^2)/9
        
        %% Finishing
        
        if SaveRaw == 1
            % Save data
            DNum1 = DNum1+1;
            save([activdir,'\Data\Droplet_',num2str(DNum1),'.mat']...
                ,'CameraRes','TBounds_x','TBounds_y','ImgSize'...
                ,'ImgDectImage','TimeAll','PDDataAll'...
                ,'Radii','FAll_n1','FAll_n2','DeltaFAll_n1'...
                ,'DeltaFAll_n2','Gamma_n2'...
                ,'eta_n2')
        end
        
        % Restart pump
        if strcmp(questdlg('Repump?'),'Yes') == 1
            fprintf(PumpObj,'RUN');
        else
            keepgoing = 0;
            break
        end
        
    end
    
    % Set up next loop iteration
    PumpRunLoop = PumpRunLoop + 1;
    DNum = DNum+1;
end

% Determine FPS
TicTocAll(TicTocAll<0.01)=[];
FPS = 1./nanmean(TicTocAll);

% Clear camera & DAQ
clear CameraStream
DAQ.outputSingleScan(0);
