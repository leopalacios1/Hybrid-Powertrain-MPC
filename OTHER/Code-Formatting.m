%-------------------------------------------------------------------------%
%                                  SETUP                                  |
%-------------------------------------------------------------------------%
% Initialize 
close all; clc; clear;
% Current Folder and Set new User Path;
fileName    = 'main.m'; 
filePath    = matlab.desktop.editor.getActiveFilename;
proyectPath = filePath(1:length(filePath)-length(char(fileName)));
userpath(proyectPath);

%-------------------------------------------------------------------------%
%                             PROGRAM SETTINGS                            %
%-------------------------------------------------------------------------%
% Define control variables to run described program blocks accordingly.

% --------------------------------- LOAD ---------------------------------%
% Load settings.  Add a new image path in the "load" program block.
% Image Settings - 

% CORRECT 1

inputImage             = 6;
imageTransform_plot    = 0;
% REMOVE 2,5, 6 
% Marker Settings
markerRealSideLength   = 10; % cm

% Camera Settings
cameraParametersPath   = 'Calibration/SmartPhoneCameraParameters.mat';

% Dictionary Validation
dictionaryPath         = 'dictionary.mat';

% -------------------------- CONTOUR EXTRACTION --------------------------%
% Contour Extraction by obtaining connected components in binarized image.
contour_extraction     = 1;
contour_plot           = 0;

% ------------------------ POLYGONAL APROXIMATION ------------------------%
% Polygonal approximation filtration of marker candidates.
poly_approx_filt       = 1;
poly_approx_filt_plot  = 0;

% ------------------------ PERIMETER FILTRATION --------------------------%
% External perimeter filtration of marker candidates.
ext_per_filt           = 1;
ext_per_filt_plot      = 0;

% -------------------------- SIZE FILTRATION -----------------------------%
% Size filtration of marker candidates
size_filt              = 1;
size_filt_plot         = 0;

% ----------------------- COLOR DIST. FILTRATION -------------------------%
% Color distribution filtration of marker candidates.
color_filt             = 1;
color_filt_plot        = 0;

% --------------------- PERSPECTIVE TRANSFORMATION -----------------------%
% 1) Marker candidate coordinate proyection onto a a square.
% 2) Binarization of pixel average across marker sizes grid squares.
% 3) Code extraction if initial validation passed.
perspective_transform  = 1; 
perspective_t_plot     = 1;

% ------------------------- MARKER VALIDATION ----------------------------%
% Marker validation against predefined orientation markers in input
% dictionary.
marker_val             = 1;

% -------------------------- POSE ESTIMATION -----------------------------%
% Camera pose estimation relative to a marker with validated global pose 
% by means of the perspective-three-point algorithm that solves our 
% perspective-n-point (PnP) problem. Define camera parameters in "LOAD" tab
pose_est               = 1; 
pose_est_plot          = 1;

%-------------------------------------------------------------------------%
%                              PROGRAM BLOCKS                             %
%-------------------------------------------------------------------------%
% TIP: Collapse outermost for-loops for easy navigation.

for i = 1 % -- LOAD ----------------------------------------------------- %

    IMAGES = {};
    imagePath = [proyectPath ,'test_images'];
    
    load(dictionaryPath);
    MARKER_SIZE = [];
    % Stored Image List
    
    image   = imread([imagePath '/test1.png']); 
    IMAGES{end+1} = image;
    MARKER_SIZE = [MARKER_SIZE, 8];
    image   = imread([imagePath '/test2.jpg']); 
    IMAGES{end+1} = image;
    MARKER_SIZE = [MARKER_SIZE, 8];
    image   = imread([imagePath '/test3.jpeg']); 
    IMAGES{end+1} = image;
    MARKER_SIZE = [MARKER_SIZE, 9];
    image   = imread([imagePath '/test4.jpg']); 
    IMAGES{end+1} = image;
    MARKER_SIZE = [MARKER_SIZE, 9];
    image   = imread([imagePath '/test5.jpeg']); 
    IMAGES{end+1} = image;
    MARKER_SIZE = [MARKER_SIZE, 9];
    image   = imread([imagePath '/test6.jpeg']); 
    IMAGES{end+1} = image;
    MARKER_SIZE = [MARKER_SIZE, 9];
    image   = imread([imagePath '/test7.jpeg']); 
    IMAGES{end+1} = image;
    MARKER_SIZE = [MARKER_SIZE, 9];
    image   = imread([imagePath '/test8.jpeg']); 
    IMAGES{end+1} = image;
    MARKER_SIZE = [MARKER_SIZE, 9];
    image   = imread([imagePath '/test9.jpeg']); 
    IMAGES{end+1} = image;
    MARKER_SIZE = [MARKER_SIZE, 9];
    image   = imread([imagePath '/test10.jpeg']); 
    IMAGES{end+1} = image;
    MARKER_SIZE = [MARKER_SIZE, 8];
    
    % Define proper marker size
    
    marker_size = MARKER_SIZE(inputImage);  
    % Load selected image
    I       = IMAGES{inputImage};

    % Image size 
    size_y  = size(I,2);
    size_x  = size(I,1);

    % Binarize image using Otsu's method
    [IBW,level,Ib] = configure(I);

    % Plot
    if imageTransform_plot
        figure;
        subplot(3,2,1)
        imshow(I);
        title('Original Image')

        subplot(3,2,2)
        imshow(IBW);
        title('Grayscale')

        subplot(3,2,3)
        imshow(Ib);
        title('Binary image')
    end
end
for i = 1 % -- CONTOUR EXTRACTION --------------------------------------- %

    if contour_extraction
        [B,L,N,A] = bwboundaries(Ib);
        if contour_plot
            subplot(3,2,5)
            imshow(IBW); 
            title('Contour Extraction')
            hold on;
            colors=['b' 'g' 'r' 'c' 'm' 'y'];
            for k=1:length(B)
              boundary = B{k};
              cidx = mod(k,length(colors))+1;
              plot(boundary(:,2), boundary(:,1),colors(cidx),'LineWidth',2);

              %randomize text position for better visibility
              rndRow = ceil(length(boundary)/(mod(rand*k,7)+1));
              col = boundary(rndRow,2); row = boundary(rndRow,1);
              h = text(col+1, row-1, num2str(L(row,col)));
              set(h,'Color',colors(cidx),'FontSize',14,'FontWeight','bold');
            end
            display(['Found ', num2str(N), ' contours']) 
        end
    end
end
for i = 1 % -- POLYGONAL APPROXIMATION FILT ----------------------------- %

    if poly_approx_filt

        [filtered,areas,rectangularity] = polygonal_approximation(B,size_x,size_y);

        if poly_approx_filt_plot
            subplot(3,2,6)
            imshow(IBW); 
            title('Polygonal approximation')
            hold on;
            colors=['b' 'g' 'r' 'c' 'm' 'y'];
            for k=1:size(rectangularity,2)
                if filtered(k)==true
                    boundary = B{k};
                    cidx = mod(k,length(colors))+1;
                    plot(boundary(:,2), boundary(:,1),colors(cidx),'LineWidth',2);
                end
            end
        end
    end
end
for i = 1 % -- EXTERNAL PERIMETER FILTRATION ---------------------------- %
    
    if ext_per_filt

        [filtered,areas] = external_perimeters(B,A,filtered,areas);

        if ext_per_filt_plot
            figure;
            subplot(3,2,1)
            imshow(IBW); 
            title('External perimeters filtration')
            hold on;
            for k=1:length(B)
              boundary = B{k};
              cidx = mod(k,length(colors))+1;
              if (filtered(k)==true)
                plot(boundary(:,2), boundary(:,1),colors(cidx),'LineWidth',2);
              end
            end
        end
    end
end
for i = 1 % -- SIZE FILTRATION ------------------------------------------ %

    if size_filt
        image_area = size_x*size_y;
        [num_markers,filtered,areas,avg_area,diff] = size_filtration(image_area,areas,filtered);

        if size_filt_plot
            subplot(3,2,2)
            bar(areas)
            title('Areas of the polygons')
            hold on
            plot(avg_area*ones(1,size(areas,2)))
            legend('Region area','Average area')

            subplot(3,2,3)
            imshow(IBW); 
            title(['Size filtration, ', num2str(num_markers), ' markers identified'])
            hold on;


            for k=1:length(B)
              boundary = B{k};
              cidx = mod(k,length(colors))+1;
              if (filtered(k)==true)
                plot(boundary(:,2), boundary(:,1),colors(cidx),'LineWidth',2);
              end
            end

            subplot(3,2,4)
            bar(diff)
        end
    end
end
for i = 1 % -- COLOR DIST. FILTRATION ----------------------------------- %

    if color_filt

        [filtered] = non_variance_filter(B,IBW,filtered,size_x,size_y);

        if color_filt_plot
            subplot(3,2,5)
            imshow(IBW);
            title(['Non variance, ', num2str(num_markers), ' markers identified'])
            hold on;
            for k=1:length(B)
              boundary = B{k};
              cidx = mod(k,length(colors))+1;
              if (filtered(k)==true)
                plot(boundary(:,2), boundary(:,1),colors(cidx),'LineWidth',2);
              end
            end
        end
    end
end
for i = 1 % -- PERSPECTIVE TRANSFORMATION ------------------------------- %
    
    if perspective_transform
        [filtered,discrete_markers,cornerList] = perspective_transform_fun(B,filtered,Ib,marker_size,perspective_t_plot);
    end
    
end
for i = 1 % -- MARKER VALIDATION ---------------------------------------- %
    markerOrientation=[];
    if marker_val
        
    [filtered,markerOrientation] = validate_markers(filtered,discrete_markers,marker_size,dictionary);
                                    
    end
    
end
for i = 1 % -- POSE ESTIMATION ------------------------------------------ %
   
    if pose_est
        % Drone Camera Parameters
        cameraParameters = importdata([proyectPath cameraParametersPath]);
        intrinsics       = cameraParameters.Intrinsics;

        [imagePointsCell,filtered] = poseEstimation(                   ...
        B,intrinsics,cornerList,filtered,markerRealSideLength,         ...
        markerOrientation, marker_size,Ib,discrete_markers,pose_est_plot);
    end

end

%-------------------------------------------------------------------------%
%                                 END                                     %
%-------------------------------------------------------------------------%



