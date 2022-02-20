function socialMonitoring()
[birdsEye,sensor] = createBirdsEyeObject();
obj = setupSystemObjects();
tracks = initializeTracks(); 
nextId = 1;
clc;
frameNumber = 1;
while ~isDone(obj.reader)
    frame = readFrame();
    birdFrame = transformFrame(frame);
    [centroids, bboxes, mask] = detectObjects(frame);

    meterCoordinates = findMeterCoordinates(centroids);
  
    birdsEyeCoordinates = findBirdsEyeCoordinates(meterCoordinates);
    
    distances = euclideanDistances(meterCoordinates);
    
    mapFrame = updateMapFrame(birdsEyeCoordinates);
    predictNewLocationsOfTracks();
    [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment();
    
    updateAssignedTracks();
    updateUnassignedTracks();
    deleteLostTracks();
    createNewTracks();
    displayTrackingResults();

    frameNumber = frameNumber + 1;
    
    
end


%% Define birdsEye object
    function [birdsEye,sensor] = createBirdsEyeObject()
        %Defining camera intrinsics
        focalLength = [309.4362 344.2161];
        principalPoint = [318.9034 257.5352];
        imageSize =  [360, 640];
        camIntrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

        %Measuring camera extrinsics
        height = 3;
        pitch = 55;
        sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);

        %Define the area of the view that needs to be transformed
        distAhead = 10;
        spaceToOneSide = 5;
        bottomOffset = 1.2;
        outView = [bottomOffset, distAhead, -spaceToOneSide, spaceToOneSide];

        %Define the birdsEye object and use it to apply the transformation on 
        % an orginial image
        outImageSize = [NaN, 250];
        birdsEye = birdsEyeView(sensor, outView, outImageSize);
    end

%% Create System Objects
    function obj = setupSystemObjects()
        obj.reader = vision.VideoFileReader('vid333.mp4');
 
        obj.maskPlayer = vision.VideoPlayer('Position', [940, 560, 700, 400]);
        set(0,'showHiddenHandles','on')
        fig_handle = gcf ;  
        fig_handle.findobj % to view all the linked objects with the vision.VideoPlayer
        ftw = fig_handle.findobj ('TooltipString', 'Maintain fit to window');   % this will search the object in the figure which has the respective 'TooltipString' parameter.
        ftw.ClickedCallback()  % execute the callback linked with this object
        obj.videoPlayer = vision.VideoPlayer('Position', [220, 560, 700, 400]);
        set(0,'showHiddenHandles','on')
        fig_handle = gcf ;  
        fig_handle.findobj % to view all the linked objects with the vision.VideoPlayer
        ftw = fig_handle.findobj ('TooltipString', 'Maintain fit to window');   % this will search the object in the figure which has the respective 'TooltipString' parameter.
        ftw.ClickedCallback()  % execute the callback linked with this object
        obj.birdPlayer = vision.VideoPlayer('Position', [220, 60, 700, 400]);
        set(0,'showHiddenHandles','on')
        fig_handle = gcf ;  
        fig_handle.findobj % to view all the linked objects with the vision.VideoPlayer
        ftw = fig_handle.findobj ('TooltipString', 'Maintain fit to window');   % this will search the object in the figure which has the respective 'TooltipString' parameter.
        ftw.ClickedCallback()  % execute the callback linked with this object
        obj.miniMap = vision.VideoPlayer('Position', [940, 60, 700, 400]);
        set(0,'showHiddenHandles','on')
        fig_handle = gcf ;  
        fig_handle.findobj % to view all the linked objects with the vision.VideoPlayer
        ftw = fig_handle.findobj ('TooltipString', 'Maintain fit to window');   % this will search the object in the figure which has the respective 'TooltipString' parameter.
        ftw.ClickedCallback()  % execute the callback linked with this object

        obj.detector = vision.ForegroundDetector('NumGaussians', 4, ...
            'NumTrainingFrames', 40, 'MinimumBackgroundRatio', 0.7);
        obj.blobAnalyser = vision.BlobAnalysis('BoundingBoxOutputPort', true, ...
            'AreaOutputPort', true, 'CentroidOutputPort', true, ...
            'MinimumBlobArea', 400);
    end

%% Initialize Tracks
    function tracks = initializeTracks()
        tracks = struct(...
            'id', {}, ...
            'bbox', {}, ...
            'kalmanFilter', {}, ...
            'age', {}, ...
            'totalVisibleCount', {}, ...
            'consecutiveInvisibleCount', {}, ...
            'center',{});
    end

%% Read a Video Frame
    function frame = readFrame()
        frame = obj.reader.step();
    end

%% Trasnform Frame
    function birdFrame = transformFrame(frame)
        birdFrame = transformImage(birdsEye, frame);
    end
        

%% Detect Objects
    function [centroids, bboxes, mask] = detectObjects(frame)
        
        % Detect foreground.
        mask = obj.detector.step(frame);
        
        % Apply morphological operations to remove noise and fill in holes.
        mask = imopen(mask, strel('rectangle', [3,3]));
        mask = imclose(mask, strel('rectangle', [6,6]));
        mask = imfill(mask, 'holes');
        
        % Perform blob analysis to find connected components.
        [~, centroids, bboxes] = obj.blobAnalyser.step(mask);
        fprintf("====================================================\n")
        fprintf("Frame number = %i\n", frameNumber)
        fprintf("Number of objects in frame = %i\n",size(centroids,1))
    end

%% Image to Vehicle (sensor)
    function meterCoordinates = findMeterCoordinates (centroids)
        meterCoordinates = imageToVehicle(sensor,centroids);
    end

%% Find coordinates of centroids in bird's eye view
    function birdsEyeCoordinates = findBirdsEyeCoordinates(meterCoordinates)
       birdsEyeCoordinates = vehicleToImage(birdsEye, meterCoordinates);
    end

%% Calculate distnaces between objects in meters
    function distances = euclideanDistances(meterCoordinates)
        distances = zeros(size(meterCoordinates,1),size(meterCoordinates,1));
        for i=1:size(meterCoordinates,1)-1
            for k=i+1:size(meterCoordinates,1)

                distances(i,k) = sqrt((meterCoordinates(i,1) - meterCoordinates(k,1))^2 ...
                     + (meterCoordinates(i,2)- meterCoordinates(k,2))^2);

                 if distances(i,k)<1
                     fprintf("Distance between two people is: %.2f meters!!\n",distances(i,k));
                 end
            end
        end
    end

%% Update Map frame
    function mapFrame = updateMapFrame(birdsEyeCoordinates)
        mapFrame = zeros(size(birdFrame));
        radius = ones(size(birdsEyeCoordinates,1) ,1) * 5;
        mapFrame = insertShape(mapFrame, 'FilledCircle',... 
            horzcat(birdsEyeCoordinates,radius), 'Color','green', ...
            'Opacity',1);
        for i = 1:size(birdsEyeCoordinates,1)-1
            for k = i+1:size(birdsEyeCoordinates,1)
                if distances(i,k)<1
                    mapFrame = insertShape(mapFrame, 'FilledCircle',... 
                        horzcat(birdsEyeCoordinates(i,:),radius), 'Color','red', ...
                        'Opacity',1);
                    mapFrame = insertShape(mapFrame, 'FilledCircle',... 
                        horzcat(birdsEyeCoordinates(k,:),radius), 'Color','red', ...
                        'Opacity',1);
                end
            end
        end
    end


%% Predict New Locations of Existing Tracks
    function predictNewLocationsOfTracks()
        for i = 1:length(tracks)
            bbox = tracks(i).bbox;
            
            % Predict the current location of the track.
            predictedCentroid = predict(tracks(i).kalmanFilter);
            
            % Shift the bounding box so that its center is at 
            % the predicted location.
            predictedCentroid = int32(predictedCentroid) - bbox(3:4) / 2;
            tracks(i).bbox = [predictedCentroid, bbox(3:4)];
        end
    end

%% Assign Detections to Tracks
    function [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment()
        
        nTracks = length(tracks);
        nDetections = size(centroids, 1);
        
        % Compute the cost of assigning each detection to each track.
        cost = zeros(nTracks, nDetections);
        for i = 1:nTracks
            cost(i, :) = distance(tracks(i).kalmanFilter, centroids);
        end
        
        % Solve the assignment problem.
        costOfNonAssignment = 20;
        [assignments, unassignedTracks, unassignedDetections] = ...
            assignDetectionsToTracks(cost, costOfNonAssignment);
    end

%% Update Assigned Tracks
    function updateAssignedTracks()
        numAssignedTracks = size(assignments, 1);
        for i = 1:numAssignedTracks
            trackIdx = assignments(i, 1);
            detectionIdx = assignments(i, 2);
            centroid = centroids(detectionIdx, :);
            bbox = bboxes(detectionIdx, :);
            
            % Correct the estimate of the object's location
            % using the new detection.
            correct(tracks(trackIdx).kalmanFilter, centroid);
            
            % Replace predicted bounding box with detected
            % bounding box.
            tracks(trackIdx).bbox = bbox;
            
            % Update track's age.
            tracks(trackIdx).age = tracks(trackIdx).age + 1;
            
            % Update visibility.
            tracks(trackIdx).totalVisibleCount = ...
                tracks(trackIdx).totalVisibleCount + 1;
            tracks(trackIdx).consecutiveInvisibleCount = 0;
        end
    end

%% Update Unassigned Tracks
% Mark each unassigned track as invisible, and increase its age by 1.

    function updateUnassignedTracks()
        for i = 1:length(unassignedTracks)
            ind = unassignedTracks(i);
            tracks(ind).age = tracks(ind).age + 1;
            tracks(ind).consecutiveInvisibleCount = ...
                tracks(ind).consecutiveInvisibleCount + 1;
        end
    end

%% Delete Lost Tracks

    function deleteLostTracks()
        if isempty(tracks)
            return;
        end
        
        invisibleForTooLong = 20;
        ageThreshold = 8;
        
        % Compute the fraction of the track's age for which it was visible.
        ages = [tracks(:).age];
        totalVisibleCounts = [tracks(:).totalVisibleCount];
        visibility = totalVisibleCounts ./ ages;
        
        % Find the indices of 'lost' tracks.
        lostInds = (ages < ageThreshold & visibility < 0.6) | ...
            [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;
        
        % Delete lost tracks.
        tracks = tracks(~lostInds);
    end

%% Create New Tracks
    function createNewTracks()
        centroids = centroids(unassignedDetections, :);
        bboxes = bboxes(unassignedDetections, :);
        
        for i = 1:size(centroids, 1)
            
            centroid = centroids(i,:);
            bbox = bboxes(i, :);
            
            % Create a Kalman filter object.
            kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
                centroid, [200, 50], [100, 25], 100);
            
            % Create a new track.
            newTrack = struct(...
                'id', nextId, ...
                'bbox', bbox, ...
                'kalmanFilter', kalmanFilter, ...
                'age', 1, ...
                'totalVisibleCount', 1, ...
                'consecutiveInvisibleCount', 0, ...
                'center', centroid);
            
            % Add it to the array of tracks.
            tracks(end + 1) = newTrack;
            
            
            % Increment the next id.
            nextId = nextId + 1;
        end
    end

%% Display Tracking Results
 
    function displayTrackingResults()
        % Convert the frame and the mask to uint8 RGB.
        frame = im2uint8(frame);
        mask = uint8(repmat(mask, [1, 1, 3])) .* 255;
        
        minVisibleCount = 8;
        if ~isempty(tracks)
              
            reliableTrackInds = ...
                [tracks(:).totalVisibleCount] > minVisibleCount;
            reliableTracks = tracks(reliableTrackInds);
            
            if ~isempty(reliableTracks)
                % Get bounding boxes.
                bboxes = cat(1, reliableTracks.bbox);
                
                % Get ids.
                ids = int32([reliableTracks(:).id]);
                
                labels = cellstr(int2str(ids'));
                
                % Calculate distance between objects in reliable tracks
%                 reliableCentroids = reliableTracks(1).center;
%                 for i=2:size(reliableTracks,2)
%                 	reliableCentroids = vertcat(reliableCentroids, ...
%                         reliableTracks(i).center);
%                 end
%    
%                 reliableCentroids
%                 meterDistance = imageToVehicle(sensor,reliableCentroids);
% 
%                 for i=1:size(meterDistance,1)-1
%                     for k=i+1:size(meterDistance,1)
% 
%                         dstnc = sqrt((meterDistance(i,1) - meterDistance(k,1))^2 ...
%                              + (meterDistance(i,2)- meterDistance(k,2))^2);
% 
%                          if dstnc < 1
%                              fprintf("Distance between object %i (%i) and %i (%i) is: %.2f meters!!\n", ...
%                                  reliableTracks(i).id,i,reliableTracks(k).id,k,dstnc);
%                          end
%                     end
%                 end

                frame = insertObjectAnnotation(frame, 'rectangle', ...
                    bboxes, labels);
                mask = insertObjectAnnotation(mask, 'rectangle', ...
                    bboxes, labels);
                
            end
        end
        obj.maskPlayer.step(mask);        
        obj.videoPlayer.step(frame);
        obj.birdPlayer.step(birdFrame);
        obj.miniMap.step(mapFrame);
%       pause(1);
        

    end
    


end
