function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize);
	% For Input and Output specifications refer to the project pdf

	import gtsam.*
	% Refer to Factor Graphs and GTSAM Introduction
	% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
	% and the examples in the library in the GTSAM toolkit. See folder
	% gtsam_toolbox/gtsam_examples
    
    %% setting up world coords
    disp('Setting up world coordinates');
    X = [0 TagSize TagSize 0];
    Y = [0 0 TagSize TagSize];
    
    %% Frame 1 to world coordinates
    disp('Calculating world coordinates of tags in frame 1');
    tag10_1 = getTag(DetAll{1},10);
    t10_x = [tag10_1(2) tag10_1(4) tag10_1(6) tag10_1(8)];
    t10_y = [tag10_1(3) tag10_1(5) tag10_1(7) tag10_1(9)];
    
    H = est_homography(t10_x, t10_y, X, Y);
    
    H = H/H(3,3);

    [KRT, T, R, T_fixed] = getKRT(H, K);

    d = DetAll{1};
    curr_ids = d(:,1);

    estimatedPoses = {}; % camera poses at each frame
    estimatedPoses{1}.R = R;
    estimatedPoses{1}.T = T;
    estimatedPoses{1}.T_fixed = T_fixed;
    estimatedWorldPoints = []; % tags in world coordinates
%     estimatedWorldPoints(1,:) = [10 0 0 TagSize 0 TagSize TagSize 0 TagSize]; %tag 10 as origin
    estimatedWorldPoints(1,:) = [10 0 0 0 0 0 0 0 0]; %tag 10 as origin
    m = 2 ;
    for t = 1: length(curr_ids)
        if curr_ids(t) ~= 10
            d1_tag = getTag(DetAll{1}, curr_ids(t));
            d1_x = [d1_tag(2) d1_tag(4) d1_tag(6) d1_tag(8)];
            d1_y = [d1_tag(3) d1_tag(5) d1_tag(7) d1_tag(9)];
            [X1, Y1] = apply_homography(KRT, d1_x(1), d1_y(1));
            [X2, Y2] = apply_homography(KRT, d1_x(2), d1_y(2));
            [X3, Y3] = apply_homography(KRT, d1_x(3), d1_y(3));
            [X4, Y4] = apply_homography(KRT, d1_x(4), d1_y(4));
            estimatedWorldPoints(m, :) = [curr_ids(t) X1 Y1 X2 Y2 X3 Y3 X4 Y4];
            m = m + 1;
        end
    end
    
    %% The rest of the frames
    disp('Calculating world coordinates of tags for the rest of the frames');
    for frame = 2:length(DetAll)
        curr_frame = DetAll{frame};
        prev_ids = estimatedWorldPoints(:,1); % all tag ids already calculated
        curr_ids = curr_frame(:,1);
        exisiting_ids = sort(intersect(curr_ids, prev_ids)); % overlapping ones
        new_ids = sort(setdiff(curr_ids, prev_ids)); % new ones
        prev_x = [];
        prev_y = [];
        curr_x = [];
        curr_y = [];
        for t = 1:length(curr_ids)
            if any(exisiting_ids(:) == curr_ids(t))
                prev_tag = getTag(estimatedWorldPoints,curr_ids(t));
                curr_tag = curr_frame(t,:);
                prev_x = [prev_x prev_tag(2) prev_tag(4) prev_tag(6) prev_tag(8)];
                prev_y = [prev_y prev_tag(3) prev_tag(5) prev_tag(7) prev_tag(9)];
                curr_x = [curr_x curr_tag(2) curr_tag(4) curr_tag(6) curr_tag(8)];
                curr_y = [curr_y curr_tag(3) curr_tag(5) curr_tag(7) curr_tag(9)];
            end
        end        
        H = est_homography(curr_x, curr_y, prev_x, prev_y);
        H = H/H(3,3);
        [KRT, T, R, T_fixed] = getKRT(H, K);
        estimatedPoses{frame}.R = R;
        estimatedPoses{frame}.T = T;
        estimatedPoses{frame}.T_fixed = T_fixed;
        for t = 1:length(curr_ids)
            if any(new_ids(:) == curr_ids(t))
                curr_tag = curr_frame(t,:);
                curr_x = [curr_tag(2) curr_tag(4) curr_tag(6) curr_tag(8)];
                curr_y = [curr_tag(3) curr_tag(5) curr_tag(7) curr_tag(9)];
                [X1, Y1] = apply_homography(KRT, curr_x(1), curr_y(1));
                [X2, Y2] = apply_homography(KRT, curr_x(2), curr_y(2));
                [X3, Y3] = apply_homography(KRT, curr_x(3), curr_y(3));
                [X4, Y4] = apply_homography(KRT, curr_x(4), curr_y(4));
                estimatedWorldPoints(length(estimatedWorldPoints)+1,:) = [curr_ids(t) X1 Y1 X2 Y2 X3 Y3 X4 Y4];
            end
        end
    end

    disp('Plotting pre-GTSAM results');
    z = [0 0 0 0 0];
    figure(1);
    hold on
    for i = 1: length(estimatedWorldPoints)
        curr_tag = estimatedWorldPoints(i,:);
        x = [curr_tag(2) curr_tag(4) curr_tag(6) curr_tag(8) curr_tag(2)];
        y = [curr_tag(3) curr_tag(5) curr_tag(7) curr_tag(9) curr_tag(3)];
        plot3(x,y,z);
    end

    for j = 1: length(estimatedPoses)
        t_temp = estimatedPoses{j}.T; 
        plot3(t_temp(1,:), t_temp(2,:), t_temp(3,:),'ro')
    end
    
    hold off;
    figure(2);
    hold on
    for i = 1: length(estimatedWorldPoints)
        curr_tag = estimatedWorldPoints(i,:);
        x = [curr_tag(2) curr_tag(4) curr_tag(6) curr_tag(8) curr_tag(2)];
        y = [curr_tag(3) curr_tag(5) curr_tag(7) curr_tag(9) curr_tag(3)];
        plot3(x,y,z);
    end

    for j = 1: length(estimatedPoses)
        t_temp = estimatedPoses{j}.T_fixed; 
        plot3(t_temp(1,:), t_temp(2,:), t_temp(3,:),'ro');
    end
    
    hold off;

    %% GTSAM
    disp("Begin GTSAM");
    % initialization
    graph = NonlinearFactorGraph;
    % addding noise
    disp("Adding Noise");
    measurementNoiseSigma = [0.1; 0.1];
    pointNoiseSigma = 0.1;
    poseNoiseSigma = [0.001; 0.001; 0.001; 0.1; 0.1; 0.1];
    odomNoise = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.1; 0.1; 0.1; 0.1]);
    measurementNoise = noiseModel.Diagonal.Sigmas(measurementNoiseSigma);
    poseNoise = noiseModel.Diagonal.Sigmas(poseNoiseSigma);
    pointNoise = noiseModel.Isotropic.Sigma(3, pointNoiseSigma);

    % adding prior factor
    disp("Adding Prior Factor and initial estimates");
    graph.add(PriorFactorPoint3(symbol('a', estimatedWorldPoints(1,1)),Point3(0,0,0),pointNoise));
    graph.add(PriorFactorPose3(symbol('x',0),Pose3(Rot3(estimatedPoses{1}.R),Point3(estimatedPoses{1}.T_fixed)),poseNoise));
    
    InitialEstimate = Values;

    K_ = Cal3_S2(K(1,1), K(2,2), K(1,2), K(1,3), K(2,3));
    for i = 1 : length(DetAll)
        currentFrame = estimatedPoses{i};
        InitialEstimate.insert(symbol('x',i-1), Pose3(Rot3(currentFrame.R),Point3(currentFrame.T_fixed)));
        if i>1
            graph.add(BetweenFactorPose3(symbol('x',i-2), symbol('x',i-1), Pose3(Rot3(eye(3)), Point3([0; 0; 0])), odomNoise));
        end
        currimageFrame = DetAll{i};
        tag_ids = currimageFrame(:,1);
        for j = 1 : length(tag_ids) 
            tag = currimageFrame(j,:);
            graph.add(GenericProjectionFactorCal3_S2(Point2(tag(2),tag(3)),measurementNoise,symbol('x',i-1),symbol('a',tag(1)),K_));
            graph.add(GenericProjectionFactorCal3_S2(Point2(tag(4),tag(5)),measurementNoise,symbol('x',i-1),symbol('b',tag(1)),K_));
            graph.add(GenericProjectionFactorCal3_S2(Point2(tag(6),tag(7)),measurementNoise,symbol('x',i-1),symbol('c',tag(1)),K_));
            graph.add(GenericProjectionFactorCal3_S2(Point2(tag(8),tag(9)),measurementNoise,symbol('x',i-1),symbol('d',tag(1)),K_));
        end
    end
    
    for i = 1 : length(estimatedWorldPoints)
        tag = estimatedWorldPoints(i,:);
        tag_id = tag(1);
        InitialEstimate.insert(symbol('a',tag_id),Point3(tag(2),tag(3),1));
        InitialEstimate.insert(symbol('b',tag_id),Point3(tag(4),tag(5),1));
        InitialEstimate.insert(symbol('c',tag_id),Point3(tag(6),tag(7),1));
        InitialEstimate.insert(symbol('d',tag_id),Point3(tag(8),tag(9),1));
    end
    
    disp("Begins GTSAM optimization");
    params = LevenbergMarquardtParams;
    params.setAbsoluteErrorTol(1e-15);
    params.setRelativeErrorTol(1e-15);
    params.setVerbosity('ERROR');
    params.setVerbosityLM('VERBOSE');
    params.setOrdering(graph.orderingCOLAMD());
    optimizer = LevenbergMarquardtOptimizer(graph, InitialEstimate, params);

    for i=1:5
        optimizer.iterate();
    end

    result = optimizer.values();

    LandMarksComputed = [];
    AllPosesComputed = {};
    m = 1;
    for i = 1 : length(estimatedWorldPoints)
        tag = estimatedWorldPoints(i,:);
        a = result.at(symbol('a',tag(1,1)));
        b = result.at(symbol('b',tag(1,1)));
        c = result.at(symbol('c',tag(1,1)));
        d = result.at(symbol('d',tag(1,1)));
        LandMarksComputed(m,:) = [tag(1,1) a.x a.y b.x b.y c.x c.y d.x d.y];
        m = m + 1;
    end
    for i = 1:length(DetAll)
        AllPosesComputed{i}.R = result.at(symbol('x',i-1)).rotation.matrix();
        AllPosesComputed{i}.T_fixed = result.at(symbol('x',i-1)).translation.vector();
    end
    
    disp("Plotting GTSAM result");
    figure(3);
    hold on
    for i = 1: length(LandMarksComputed)
        curr_tag = LandMarksComputed(i,:);
        x = [curr_tag(2) curr_tag(4) curr_tag(6) curr_tag(8) curr_tag(2)];
        y = [curr_tag(3) curr_tag(5) curr_tag(7) curr_tag(9) curr_tag(3)];
        plot3(x,y,z);
    end

    for j = 1: length(AllPosesComputed)
        t_temp = AllPosesComputed{j}.T_fixed; 
        plot3(t_temp(1), t_temp(2), t_temp(3),'ro');
    end
    
    hold off;

end

function tag = getTag(frame, id)
    found = false;
    for i = 1: length(frame)
        if frame(i,1)==id
            found = true;
            break
        end
    end
    if found == true
        tag = frame(i,:);
    else
        tag = -100;
    end
end

function [KRT, T, R, T_fixed] = getKRT(H, K)

    K_inv = inv(K); % 3x3
    
    Kinv_H = K_inv * H; % 3x3
    h3 = Kinv_H(:,3);
    Kinv_H(:,3) = cross(Kinv_H(:,1),Kinv_H(:,2));
    [U, S, V] = svd(Kinv_H);
    Vt = V';
    UVt = U*Vt; % 3x3
    dUVt = det(UVt);
    midMatrix = [1 0 0; 0 1 0; 0 0 dUVt];
    R = U * midMatrix *Vt; % 3x3

    T = h3/norm(Kinv_H(:,1)); % 1x3
    
    RT = [R(:,1) R(:,2) T]; % 3x3

    KRT = inv(K*RT);
    T_fixed = -R' * T;
end