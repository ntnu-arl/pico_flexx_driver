function royale_LEVEL2_sample2()
%ROYALE_LEVEL2_SAMPLE2 - royale LEVEL 2 example #2: raw data acquisition

% retrieve royale version information
royaleVersion = royale.getVersion();
fprintf('* royale version: %s\n',royaleVersion);

% the camera manager will query for a connected camera
manager = royale.CameraManager(activationCode);
camlist = manager.getConnectedCameraList();

fprintf('* Cameras found: %d\n',numel(camlist));
cellfun(@(cameraId)...
    fprintf('    %s\n',cameraId),...
    camlist);

if (~isempty(camlist))
    % this represents the main camera device object
    cameraDevice = manager.createCamera(camlist{1});
else
    error(['Please make sure that a supported camera is plugged in, all drivers are ',...
        'installed, and you have proper USB permission']);
end

% the camera device is now available and CameraManager can be deallocated here
delete(manager);

% activating raw data mode
fprintf('* activating raw data mode\n');
cameraDevice.setCallbackData(royale.CallbackData.Raw);

% IMPORTANT: call the initialize method before working with the camera device
cameraDevice.initialize();

% retrieve valid use cases
UseCases=cameraDevice.getUseCases();
fprintf('Use cases: %d\n',numel(UseCases));
fprintf('    %s\n',UseCases{:});
fprintf('====================================\n');

if (numel(UseCases) == 0)
    error('No use case available');
end
    
% % set use case
% UseCase=UseCases{1};

% set use case interactively
UseCaseSelection=listdlg(...
    'Name','Operation Mode',...
    'PromptString','Choose operation mode:',...
    'ListString',UseCases,...
    'SelectionMode','single',...
    'ListSize',[200,200]);
if isempty(UseCaseSelection)
    return;
end
UseCase=UseCases{UseCaseSelection};

cameraDevice.setUseCase(UseCase);


CameraAccessLevel = cameraDevice.getAccessLevel();
fprintf('* Camera access level: %d\n',CameraAccessLevel);

% preview camera
fprintf('* Starting preview. Close figure to exit...\n');
% start capture mode
cameraDevice.startCapture();

% initialize preview figure
UseCase = cameraDevice.getCurrentUseCase();

FigureTitle = ['Preview: ',cameraDevice.getId(),' @ ', UseCase];
hFig=figure('Name',...
    ['Preview: ',cameraDevice.getId(),' @ ', UseCase],...
    'IntegerHandle','off','NumberTitle','off');
colormap(jet(256));
TID = tic();
last_toc = toc(TID);
iFrame = 0;
while (ishandle(hFig))
    % retrieve data from camera
    [~, ~, RawData] = cameraDevice.getData();
    
    iFrame = iFrame + 1;
    if (mod(iFrame,10) == 0)
        this_toc=toc(TID);
        fprintf('FPS = %.2f\n',10/(this_toc-last_toc));
        last_toc=this_toc;
    end
    
    %%% notice: figures are slow.
    %%% For higher FPS (e.g. 45), do not display every frame.
    %%% e.g. by doing here:
    % if (mod(iFrame,5) ~= 0);continue;end;
    
    N_Raw = numel(RawData.rawData);
    sp_columns_raw=ceil(sqrt(N_Raw));
    sp_rows_raw=ceil(N_Raw/sp_columns_raw);
   
    % visualize data
    set(0,'CurrentFigure',hFig);
    for iRaw=1:N_Raw
        subplot(sp_rows_raw,sp_columns_raw,iRaw);
        my_image(RawData.rawData{iRaw},['Raw ',num2str(iRaw,'%d')]);
        set(gcf,'Name', [FigureTitle,' @ Temp = ',num2str(RawData.illuminationTemperature,'%.2f'),'°C']);
    end

    drawnow;
end

% stop capture mode
fprintf('* Stopping capture mode...\n');
cameraDevice.stopCapture();

fprintf('* ...done!\n');
end

function my_image(CData,Name)
% convenience function for faster display refresh:
%  only update 'CData' on successive image calls
%  (does not update title or change resolution!)
if ~isappdata(gca,'my_imagehandle')
    my_imagehandle = imagesc(CData);
    axis image
    title(Name);
    setappdata(gca,'my_imagehandle',my_imagehandle);
else
    my_imagehandle = getappdata(gca,'my_imagehandle');
    set(my_imagehandle,'CData',CData);
end
end
