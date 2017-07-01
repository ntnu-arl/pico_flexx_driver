function royale_LEVEL2_sample3()
%ROYALE_LEVEL2_SAMPLE3 - royale LEVEL 2 example #3

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

cameraDevice.setCallbackData(royale.CallbackData.Intermediate);

% preview camera
fprintf('* Starting preview. Close figure to exit...\n');
% start capture mode
cameraDevice.startCapture();

% initialize preview figure
UseCase = cameraDevice.getCurrentUseCase();
hFig=figure('Name',...
    ['Preview: ',cameraDevice.getId(),' @ ', UseCase],...
    'IntegerHandle','off','NumberTitle','off');
colormap(jet(256));
TID = tic();
last_toc = toc(TID);
iFrame = 0;
while (ishandle(hFig))
    % retrieve data from camera
    [~, IntermediateData] = cameraDevice.getData();
    
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
    
    % visualize data
    set(0,'CurrentFigure',hFig);
    
    N_Seq=numel(IntermediateData.distance);
    for iSeq=1:N_Seq
        
        % SDK values are unsigned integers
        % => convert to double for real number represenation
        Fmod_String=[num2str(...
            double(IntermediateData.modulationFrequencies(iSeq))*1e-6...
            ,'%.2f'),' MHz'];
        
        subplot(N_Seq,3,3*(iSeq-1)+1);
        my_image(IntermediateData.distance{iSeq},['distance @ ', Fmod_String]);

        subplot(N_Seq,3,3*(iSeq-1)+2);
        my_image(IntermediateData.amplitude{iSeq},['amplitude @ ', Fmod_String]);
        
        subplot(N_Seq,3,3*(iSeq-1)+3);
        my_image(IntermediateData.intensity{iSeq},['intensity',num2str(iSeq,'%d')]);
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
