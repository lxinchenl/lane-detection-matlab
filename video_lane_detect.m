function video_lane_detect()

    % 创建主窗口
    fig = figure('Name', '车道线检测系统', 'NumberTitle', 'off', ...
        'Position', [50, 50, 1500, 850], ...
        'CloseRequestFcn', @closeGUI, ...
        'Resize', 'on');
    
    % ========== 全局变量 ==========
    data = struct();
    data.videoFile = '';
    data.videoSrc = [];
    data.isRunning = false;
    data.useKalman = false;
    data.leftKalman = [];
    data.rightKalman = [];
    data.frameCount = 0;
    data.timerObj = [];
    data.videoWriter = [];
    data.isRecording = false;
    
    % ========== 参数设置（沿用当前设置）==========
    data.params.ROI_PERCENT = 0.4;
    data.params.BINARY_THRESH = 0.5;
    data.params.EDGE_METHOD = 'Otsu';
    data.params.HOUGH_PEAKS = 10;
    data.params.HOUGH_THRESHOLD = 0.3;
    data.params.HOUGH_FILLGAP = 20;
    data.params.HOUGH_MINLENGTH = 30;
    data.params.ANGLE_MIN = 20;
    data.params.ANGLE_MAX = 70;
    data.params.MIN_LINE_LENGTH = 40;
    data.params.LEFT_MIN_X = 0.2;
    data.params.LEFT_MAX_X = 0.9;
    data.params.RIGHT_MIN_X = 1.1;
    data.params.RIGHT_MAX_X = 0.8;
    data.params.SCORE_LENGTH_WEIGHT = 2;
    data.params.SCORE_DISTANCE_WEIGHT = 0.5;
    data.params.SCORE_ANGLE_WEIGHT = 0.3;
    data.params.KALMAN_Q = 0.1;
    data.params.KALMAN_R = 10;
    data.params.ANGLE_THRESHOLD = 5;
    
    controlPanel = uipanel('Parent', fig, 'Title', '控制面板', ...
        'Position', [0.01, 0.35, 0.23, 0.63], ...
        'FontSize', 11, 'FontWeight', 'bold');
    
    % 加载视频按钮
    btnLoad = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', '加载视频', 'Position', [15, 480, 180, 35], ...
        'FontSize', 11, 'Callback', @loadVideo, ...
        'BackgroundColor', [0.3, 0.6, 0.9], 'ForegroundColor', 'white');
    
    % 视频文件显示
    txtFile = uicontrol('Parent', controlPanel, 'Style', 'text', ...
        'String', '未选择视频', 'Position', [15, 445, 300, 25], ...
        'FontSize', 9, 'HorizontalAlignment', 'left');
    
    % 分隔线
    uicontrol('Parent', controlPanel, 'Style', 'text', ...
        'String', '______________________________', ...
        'Position', [15, 420, 300, 20], 'ForegroundColor', [0.7, 0.7, 0.7]);
    
    % 开始/停止检测按钮
    btnStart = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', '开始检测', 'Position', [15, 370, 180, 40], ...
        'FontSize', 12, 'FontWeight', 'bold', 'Callback', @toggleDetection, ...
        'BackgroundColor', [0.2, 0.7, 0.3], 'ForegroundColor', 'white', ...
        'Enable', 'off');
    
    % 单步检测按钮
    btnStep = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', '单步检测', 'Position', [15, 325, 180, 35], ...
        'FontSize', 10, 'Callback', @stepDetection, ...
        'Enable', 'off');
    
    % 导出视频按钮
    btnExport = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', '导出结果视频', 'Position', [15, 280, 180, 35], ...
        'FontSize', 10, 'Callback', @exportVideo, ...
        'BackgroundColor', [0.9, 0.5, 0.2], 'ForegroundColor', 'white', ...
        'Enable', 'off');
    
    % 分隔线
    uicontrol('Parent', controlPanel, 'Style', 'text', ...
        'String', '______________________________', ...
        'Position', [15, 250, 300, 20], 'ForegroundColor', [0.7, 0.7, 0.7]);
    
    % 卡尔曼滤波开关
    chkKalman = uicontrol('Parent', controlPanel, 'Style', 'checkbox', ...
        'String', '启用卡尔曼滤波', 'Position', [15, 220, 150, 25], ...
        'FontSize', 10, 'Callback', @toggleKalman);
    
    % ========== 卡尔曼参数调节区 ==========
    kalmanPanel = uipanel('Parent', controlPanel, 'Title', '卡尔曼参数', ...
        'Position', [0.05, 0.08, 0.9, 0.28], 'FontSize', 9);
    
    % Q参数（过程噪声）
    uicontrol('Parent', kalmanPanel, 'Style', 'text', ...
        'String', '过程噪声 Q:', 'Position', [10, 110, 80, 20], ...
        'FontSize', 9, 'HorizontalAlignment', 'left');
    
    sliderQ = uicontrol('Parent', kalmanPanel, 'Style', 'slider', ...
        'Min', 0.001, 'Max', 1, 'Value', data.params.KALMAN_Q, ...
        'Position', [10, 90, 280, 20], 'Callback', @updateQ);
    
    txtQ = uicontrol('Parent', kalmanPanel, 'Style', 'text', ...
        'String', sprintf('%.3f', data.params.KALMAN_Q), ...
        'Position', [10, 70, 80, 18], 'FontSize', 9);
    
    % R参数（测量噪声）
    uicontrol('Parent', kalmanPanel, 'Style', 'text', ...
        'String', '测量噪声 R:', 'Position', [10, 45, 80, 20], ...
        'FontSize', 9, 'HorizontalAlignment', 'left');
    
    sliderR = uicontrol('Parent', kalmanPanel, 'Style', 'slider', ...
        'Min', 1, 'Max', 500, 'Value', data.params.KALMAN_R, ...
        'Position', [10, 25, 280, 20], 'Callback', @updateR);
    
    txtR = uicontrol('Parent', kalmanPanel, 'Style', 'text', ...
        'String', sprintf('%.1f', data.params.KALMAN_R), ...
        'Position', [10, 5, 80, 18], 'FontSize', 9);
    
    % 原始视频显示
    axOriginal = axes('Parent', fig, 'Position', [0.26, 0.52, 0.72, 0.46]);
    title(axOriginal, '原始检测');
    axis(axOriginal, 'off');
    imshow(zeros(480, 640, 3, 'uint8'), 'Parent', axOriginal);
    
    % 卡尔曼滤波结果显示（或仅显示结果）
    axFiltered = axes('Parent', fig, 'Position', [0.26, 0.05, 0.72, 0.46]);
    title(axFiltered, '检测结果');
    axis(axFiltered, 'off');
    imshow(zeros(480, 640, 3, 'uint8'), 'Parent', axFiltered);
    
    infoPanel = uipanel('Parent', fig, 'Title', '状态信息', ...
        'Position', [0.01, 0.01, 0.23, 0.33], 'FontSize', 10);
    
    txtInfo = uicontrol('Parent', infoPanel, 'Style', 'text', ...
        'String', '就绪', 'Position', [10, 10, 300, 250], ...
        'FontSize', 9, 'HorizontalAlignment', 'left');
    
    legendPanel = uipanel('Parent', fig, 'Title', '图例', ...
        'Position', [0.26, 0.01, 0.72, 0.03], 'FontSize', 9);
    
    uicontrol('Parent', legendPanel, 'Style', 'text', ...
        'String', '原始检测线', 'Position', [50, 0, 100, 20], ...
        'ForegroundColor', 'blue', 'FontWeight', 'bold');
    uicontrol('Parent', legendPanel, 'Style', 'text', ...
        'String', '滤波后线', 'Position', [200, 0, 100, 20], ...
        'ForegroundColor', 'red', 'FontWeight', 'bold');
    
    
    function loadVideo(~, ~)
        [file, path] = uigetfile({'*.mp4;*.avi;*.mov', '视频文件 (*.mp4, *.avi, *.mov)'}, '选择视频文件');
        if isequal(file, 0)
            return;
        end
        
        data.videoFile = fullfile(path, file);
        txtFile.String = file;
        
        % 打开视频
        try
            if ~isempty(data.videoSrc)
                delete(data.videoSrc);
            end
            data.videoSrc = VideoReader(data.videoFile);
            data.frameCount = 0;
            data.leftKalman = [];
            data.rightKalman = [];
            
            % 显示第一帧
            frame = readFrame(data.videoSrc);
            imshow(frame, 'Parent', axOriginal);
            imshow(frame, 'Parent', axFiltered);
            
            btnStart.Enable = 'on';
            btnStep.Enable = 'on';
            btnExport.Enable = 'on';
            updateInfo(sprintf('视频已加载\n分辨率: %dx%d\n总帧数: %d', ...
                data.videoSrc.Width, data.videoSrc.Height, data.videoSrc.NumFrames));
        catch ME
            errordlg(['无法加载视频: ' ME.message], '错误');
        end
    end
    
    function toggleKalman(src, ~)
        data.useKalman = src.Value;
        if data.useKalman
            kalmanPanel.ForegroundColor = [0.2, 0.7, 0.3];
        else
            kalmanPanel.ForegroundColor = [0.3, 0.3, 0.3];
        end
    end
    
    function updateQ(src, ~)
        data.params.KALMAN_Q = src.Value;
        txtQ.String = sprintf('%.3f', data.params.KALMAN_Q);
        % 重置卡尔曼滤波器以应用新参数
        data.leftKalman = [];
        data.rightKalman = [];
    end
    
    function updateR(src, ~)
        data.params.KALMAN_R = src.Value;
        txtR.String = sprintf('%.1f', data.params.KALMAN_R);
        % 重置卡尔曼滤波器以应用新参数
        data.leftKalman = [];
        data.rightKalman = [];
    end
    
    function toggleDetection(~, ~)
        if data.isRunning
            stopDetection();
        else
            startDetection();
        end
    end
    
    function startDetection()
        if isempty(data.videoSrc)
            errordlg('请先加载视频', '错误');
            return;
        end
        
        data.isRunning = true;
        btnStart.String = '停止检测';
        btnStart.BackgroundColor = [0.9, 0.3, 0.3];
        btnLoad.Enable = 'off';
        btnStep.Enable = 'off';
        btnExport.Enable = 'off';
        
        % 重置视频到开头
        data.videoSrc.CurrentTime = 0;
        data.frameCount = 0;
        data.leftKalman = [];
        data.rightKalman = [];
        
        % 创建定时器用于连续播放
        data.timerObj = timer('ExecutionMode', 'fixedRate', ...
            'Period', 0.033, ...  % ~30fps
            'TimerFcn', @processFrame);
        start(data.timerObj);
    end
    
    function stopDetection()
        data.isRunning = false;
        if ~isempty(data.timerObj) && isvalid(data.timerObj)
            stop(data.timerObj);
            delete(data.timerObj);
            data.timerObj = [];
        end
        
        btnStart.String = '开始检测';
        btnStart.BackgroundColor = [0.2, 0.7, 0.3];
        btnLoad.Enable = 'on';
        btnStep.Enable = 'on';
        btnExport.Enable = 'on';
    end
    
    function stepDetection(~, ~)
        if isempty(data.videoSrc)
            return;
        end
        
        if ~hasFrame(data.videoSrc)
            data.videoSrc.CurrentTime = 0;
            data.frameCount = 0;
        end
        
        processFrame();
    end
    
    function exportVideo(~, ~)
        if isempty(data.videoSrc)
            errordlg('请先加载视频', '错误');
            return;
        end
        
        % 选择保存路径
        [file, path] = uiputfile({'*.mp4', 'MP4视频 (*.mp4)'; '*.avi', 'AVI视频 (*.avi)'}, ...
            '保存结果视频', 'lane_detection_result.mp4');
        if isequal(file, 0)
            return;
        end
        
        outputFile = fullfile(path, file);
        
        % 创建视频写入器
        try
            data.videoWriter = VideoWriter(outputFile, 'MPEG-4');
            data.videoWriter.FrameRate = 30;
            data.videoWriter.Quality = 95;
            open(data.videoWriter);
            data.isRecording = true;
        catch
            % 如果MPEG-4不支持，尝试Motion JPEG AVI
            data.videoWriter = VideoWriter(outputFile, 'Motion JPEG AVI');
            data.videoWriter.FrameRate = 30;
            data.videoWriter.Quality = 95;
            open(data.videoWriter);
            data.isRecording = true;
        end
        
        % 重置视频到开头
        data.videoSrc.CurrentTime = 0;
        data.frameCount = 0;
        data.leftKalman = [];
        data.rightKalman = [];
        
        % 禁用其他按钮
        btnLoad.Enable = 'off';
        btnStart.Enable = 'off';
        btnStep.Enable = 'off';
        btnExport.String = '导出中...';
        btnExport.BackgroundColor = [0.5, 0.5, 0.5];
        
        updateInfo('正在导出视频...');
        
        % 处理所有帧
        while hasFrame(data.videoSrc)
            frame = readFrame(data.videoSrc);
            data.frameCount = data.frameCount + 1;
            [imgHeight, imgWidth, ~] = size(frame);
            
            % 计算ROI
            roiStart = round(imgHeight * (1 - data.params.ROI_PERCENT));
            
            % 处理帧
            [~, rawLeftLane, rawRightLane] = processFrameOptimized(frame, roiStart, data.params);
            
            % 卡尔曼滤波
            filteredLeftLane = rawLeftLane;
            filteredRightLane = rawRightLane;
            
            if data.useKalman
                [filteredLeftLane, data.leftKalman] = kalmanFilterLane(...
                    rawLeftLane, data.leftKalman, roiStart, imgHeight, ...
                    data.params.KALMAN_Q, data.params.KALMAN_R);
                [filteredRightLane, data.rightKalman] = kalmanFilterLane(...
                    rawRightLane, data.rightKalman, roiStart, imgHeight, ...
                    data.params.KALMAN_Q, data.params.KALMAN_R);
            end
            
            % 绘制结果（使用下半部分的显示逻辑）
            if data.useKalman
                frameResult = drawLanesWithStyle(frame, rawLeftLane, rawRightLane, roiStart, 'blue');
                frameResult = drawLanesWithStyle(frameResult, filteredLeftLane, filteredRightLane, roiStart, 'red');
            else
                frameResult = drawLanesWithStyle(frame, rawLeftLane, rawRightLane, roiStart, 'red');
            end
            
            % 写入视频
            writeVideo(data.videoWriter, frameResult);
            
            % 更新显示（每10帧更新一次界面）
            if mod(data.frameCount, 10) == 0
                imshow(frameResult, 'Parent', axFiltered);
                title(axFiltered, sprintf('导出中... 帧 %d', data.frameCount));
                drawnow limitrate;
                updateInfo(sprintf('导出中...\n已处理: %d 帧', data.frameCount));
            end
        end
        
        % 关闭视频写入器
        close(data.videoWriter);
        data.videoWriter = [];
        data.isRecording = false;
        
        % 恢复按钮状态
        btnLoad.Enable = 'on';
        btnStart.Enable = 'on';
        btnStep.Enable = 'on';
        btnExport.String = '导出结果视频';
        btnExport.BackgroundColor = [0.9, 0.5, 0.2];
        
        updateInfo(sprintf('导出完成!\n保存至: %s\n总帧数: %d', outputFile, data.frameCount));
        msgbox(sprintf('视频导出完成!\n保存至: %s', outputFile), '导出成功');
    end
    
    function processFrame(~, ~)
        if ~hasFrame(data.videoSrc)
            if data.isRunning
                stopDetection();
            end
            return;
        end
        
        frame = readFrame(data.videoSrc);
        data.frameCount = data.frameCount + 1;
        [imgHeight, imgWidth, ~] = size(frame);
        
        % 计算ROI
        roiStart = round(imgHeight * (1 - data.params.ROI_PERCENT));
        
        % 处理帧 - 获取原始检测结果
        [~, rawLeftLane, rawRightLane] = processFrameOptimized(frame, roiStart, data.params);
        
        % 绘制原始检测结果（上半部分）
        if data.useKalman
            % 启用卡尔曼时，原始检测用不透明蓝色
            frameOriginal = drawLanesWithStyle(frame, rawLeftLane, rawRightLane, roiStart, ...
                'blue');  % 不透明蓝色
        else
            % 不启用卡尔曼时，用不透明红线
            frameOriginal = drawLanesWithStyle(frame, rawLeftLane, rawRightLane, roiStart, ...
                'red');  % 不透明红色
        end
        
        % 卡尔曼滤波处理
        filteredLeftLane = rawLeftLane;
        filteredRightLane = rawRightLane;
        
        if data.useKalman
            [filteredLeftLane, data.leftKalman] = kalmanFilterLane(...
                rawLeftLane, data.leftKalman, roiStart, imgHeight, ...
                data.params.KALMAN_Q, data.params.KALMAN_R);
            [filteredRightLane, data.rightKalman] = kalmanFilterLane(...
                rawRightLane, data.rightKalman, roiStart, imgHeight, ...
                data.params.KALMAN_Q, data.params.KALMAN_R);
        end
        
        % 绘制最终结果（下半部分）
        if data.useKalman
            % 启用卡尔曼时，下半部分显示：原始检测（蓝）+ 滤波后（红）
            frameFiltered = drawLanesWithStyle(frame, rawLeftLane, rawRightLane, roiStart, ...
                'blue');  % 不透明蓝色（原始）
            frameFiltered = drawLanesWithStyle(frameFiltered, filteredLeftLane, filteredRightLane, roiStart, ...
                'red');  % 不透明红色（滤波后）
        else
            % 不启用卡尔曼时，只显示红色结果
            frameFiltered = frameOriginal;
        end
        
        % 更新显示
        imshow(frameOriginal, 'Parent', axOriginal);
        title(axOriginal, sprintf('原始检测 (帧 %d)', data.frameCount));
        
        imshow(frameFiltered, 'Parent', axFiltered);
        if data.useKalman
            title(axFiltered, sprintf('卡尔曼滤波结果 (帧 %d)', data.frameCount));
        else
            title(axFiltered, sprintf('检测结果 (帧 %d)', data.frameCount));
        end
        
        drawnow limitrate;
        
        % 更新状态信息
        infoStr = sprintf('帧: %d\n原始左线: %s\n原始右线: %s', ...
            data.frameCount, ...
            laneStatus(rawLeftLane), laneStatus(rawRightLane));
        if data.useKalman
            infoStr = [infoStr sprintf('\n滤波左线: %s\n滤波右线: %s', ...
                laneStatus(filteredLeftLane), laneStatus(filteredRightLane))];
            infoStr = [infoStr sprintf('\n\n卡尔曼参数:\nQ=%.3f, R=%.1f', ...
                data.params.KALMAN_Q, data.params.KALMAN_R)];
        end
        updateInfo(infoStr);
    end
    
    function str = laneStatus(lane)
        if isempty(lane)
            str = '未检测到';
        else
            str = sprintf('角度=%.1f°', lane.angle);
        end
    end
    
    function updateInfo(str)
        txtInfo.String = str;
    end
    
    function closeGUI(~, ~)
        stopDetection();
        if ~isempty(data.videoWriter)
            close(data.videoWriter);
        end
        if ~isempty(data.videoSrc)
            delete(data.videoSrc);
        end
        delete(fig);
    end
end

%% ==================== 处理函数 ====================

function [processedFrame, leftLane, rightLane] = processFrameOptimized(frame, roiStart, params)
    % 处理单帧图像，检测车道线
    
    % 转换为灰度图
    grayImg = rgb2gray(frame);
    
    % ROI裁剪
    roiImg = grayImg(roiStart:end, :);
    
    % 二值化
    binaryImg = im2bw(roiImg, params.BINARY_THRESH);
    
    % 边缘检测
    switch params.EDGE_METHOD
        case 'Sobel'
            edgeImg = edge(binaryImg, 'Sobel');
        case 'Canny'
            edgeImg = edge(binaryImg, 'Canny');
        case 'Otsu'
            mag_norm = mat2gray(double(binaryImg));
            th_otsu = graythresh(mag_norm);
            edgeImg = mag_norm >= th_otsu;
        otherwise
            edgeImg = edge(binaryImg, 'Sobel');
    end
    
    % 霍夫变换
    [H, theta, rho] = hough(edgeImg);
    peaks = houghpeaks(H, params.HOUGH_PEAKS, 'Threshold', params.HOUGH_THRESHOLD * max(H(:)));
    lines = houghlines(edgeImg, theta, rho, peaks, ...
        'FillGap', params.HOUGH_FILLGAP, 'MinLength', params.HOUGH_MINLENGTH);
    
    % 筛选车道线
    [leftLane, rightLane] = selectLanesOptimized(lines, size(edgeImg), params);
    
    processedFrame = edgeImg;
end

function [leftLane, rightLane] = selectLanesOptimized(lines, imgSize, params)
    % 筛选车道线
    
    leftLane = [];
    rightLane = [];
    
    if isempty(lines)
        return;
    end
    
    imgWidth = imgSize(2);
    imgCenterX = imgWidth / 2;
    
    % 提取直线信息
    lineInfo = struct('point1', {}, 'point2', {}, 'slope', {}, 'angle', {}, ...
                      'midpoint', {}, 'length', {}, 'distanceToCenter', {}, ...
                      'side', {});
    
    for k = 1:length(lines)
        p1 = lines(k).point1;
        p2 = lines(k).point2;
        
        dx = p2(1) - p1(1);
        dy = p2(2) - p1(2);
        
        if abs(dx) < 1e-6
            slope = inf;
            angle = 90;
        else
            slope = dy / dx;
            angle = atand(slope);
        end
        
        midpoint = [(p1(1) + p2(1))/2, (p1(2) + p2(2))/2];
        len = norm([dx, dy]);
        
        lineInfo(k).point1 = p1;
        lineInfo(k).point2 = p2;
        lineInfo(k).slope = slope;
        lineInfo(k).angle = angle;
        lineInfo(k).midpoint = midpoint;
        lineInfo(k).length = len;
        lineInfo(k).distanceToCenter = abs(midpoint(1) - imgCenterX);
        
        if slope < -0.3 && slope > -5
            lineInfo(k).side = 'left';
        elseif slope > 0.3 && slope < 5
            lineInfo(k).side = 'right';
        else
            lineInfo(k).side = 'unknown';
        end
    end
    
    % 过滤和分类
    filteredLines = [];
    for k = 1:length(lines)
        angle = lineInfo(k).angle;
        len = lineInfo(k).length;
        side = lineInfo(k).side;
        
        if (abs(angle) >= params.ANGLE_MIN && abs(angle) <= params.ANGLE_MAX) && ...
           len > params.MIN_LINE_LENGTH && ~strcmp(side, 'unknown')
            filteredLines = [filteredLines; lineInfo(k)];
        end
    end
    
    leftLines = filteredLines(strcmp({filteredLines.side}, 'left'));
    rightLines = filteredLines(strcmp({filteredLines.side}, 'right'));
    
    % 选择最优车道线
    leftLane = selectBestLane(leftLines, imgWidth, imgCenterX, params, 'left');
    rightLane = selectBestLane(rightLines, imgWidth, imgCenterX, params, 'right');
    
    % 备选方案
    if isempty(leftLane) && ~isempty(rightLane)
        for k = 1:length(leftLines)
            if leftLines(k).midpoint(1) < imgCenterX
                leftLane = leftLines(k);
                break;
            end
        end
    elseif isempty(rightLane) && ~isempty(leftLane)
        for k = 1:length(rightLines)
            if rightLines(k).midpoint(1) > imgCenterX
                rightLane = rightLines(k);
                break;
            end
        end
    end
end

function bestLane = selectBestLane(lines, imgWidth, imgCenterX, params, side)
    bestLane = [];
    if isempty(lines)
        return;
    end
    
    [~, sortIdx] = sort([lines.distanceToCenter], 'ascend');
    sortedLines = lines(sortIdx);
    
    validLines = [];
    for k = 1:length(sortedLines)
        midX = sortedLines(k).midpoint(1);
        if strcmp(side, 'left')
            if midX < imgCenterX * params.LEFT_MAX_X && midX > imgWidth * params.LEFT_MIN_X
                validLines = [validLines; sortedLines(k)];
            end
        else
            if midX > imgCenterX * params.RIGHT_MIN_X && midX < imgWidth * params.RIGHT_MAX_X
                validLines = [validLines; sortedLines(k)];
            end
        end
    end
    
    if ~isempty(validLines)
        bestScore = -inf;
        for k = 1:length(validLines)
            len = validLines(k).length;
            dist = validLines(k).distanceToCenter;
            angle = validLines(k).angle;
            
            score = len * params.SCORE_LENGTH_WEIGHT - ...
                    dist * params.SCORE_DISTANCE_WEIGHT + ...
                    abs(angle) * params.SCORE_ANGLE_WEIGHT;
            
            if score > bestScore
                bestScore = score;
                bestLane = validLines(k);
            end
        end
    end
end

function frame = drawLanesWithStyle(frame, leftLane, rightLane, roiStart, color)
    % 绘制车道线
    
    [imgHeight, imgWidth, ~] = size(frame);
    
    % 线宽设置（加粗）
    lineWidth = 8;
    
    % 绘制左车道线
    if ~isempty(leftLane)
        p1 = [leftLane.point1(1), leftLane.point1(2) + roiStart];
        p2 = [leftLane.point2(1), leftLane.point2(2) + roiStart];
        
        slope = leftLane.slope;
        if abs(slope) > 0.01 && abs(slope) < 100
            y_bottom = imgHeight;
            x_bottom = p2(1) + (y_bottom - p2(2)) / slope;
            y_top = roiStart;
            x_top = p1(1) + (y_top - p1(2)) / slope;
            
            frame = insertShape(frame, 'Line', ...
                [x_top, y_top, x_bottom, y_bottom], ...
                'Color', color, 'LineWidth', lineWidth);
        else
            frame = insertShape(frame, 'Line', ...
                [p1(1), p1(2), p2(1), p2(2)], ...
                'Color', color, 'LineWidth', lineWidth);
        end
    end
    
    % 绘制右车道线
    if ~isempty(rightLane)
        p1 = [rightLane.point1(1), rightLane.point1(2) + roiStart];
        p2 = [rightLane.point2(1), rightLane.point2(2) + roiStart];
        
        slope = rightLane.slope;
        if abs(slope) > 0.01 && abs(slope) < 100
            y_bottom = imgHeight;
            x_bottom = p2(1) + (y_bottom - p2(2)) / slope;
            y_top = roiStart;
            x_top = p1(1) + (y_top - p1(2)) / slope;
            
            frame = insertShape(frame, 'Line', ...
                [x_top, y_top, x_bottom, y_bottom], ...
                'Color', color, 'LineWidth', lineWidth);
        else
            frame = insertShape(frame, 'Line', ...
                [p1(1), p1(2), p2(1), p2(2)], ...
                'Color', color, 'LineWidth', lineWidth);
        end
    end
end

function [filteredLane, kalman] = kalmanFilterLane(currLane, kalman, roiStart, imgHeight, Q, R)
    % 卡尔曼滤波
    
    if isempty(currLane)
        if ~isempty(kalman)
            kalman.x = kalman.A * kalman.x;
            kalman.P = kalman.A * kalman.P * kalman.A' + kalman.Q;
            filteredLane = kalman.lane;
        else
            filteredLane = [];
        end
        return;
    end
    
    slope = currLane.slope;
    if abs(slope) > 0.01 && abs(slope) < 100
        y_bottom = imgHeight;
        x_bottom = currLane.point2(1) + (y_bottom - currLane.point2(2) - roiStart) / slope;
        y_top = roiStart;
        x_top = currLane.point1(1) + (y_top - currLane.point1(2) - roiStart) / slope;
    else
        x_bottom = currLane.point2(1);
        x_top = currLane.point1(1);
    end
    
    z = [x_bottom; x_top];
    
    if isempty(kalman)
        kalman.x = z;
        kalman.P = eye(2) * 1000;
        kalman.A = eye(2);
        kalman.H = eye(2);
        kalman.Q = eye(2) * Q;
        kalman.R = eye(2) * R;
        kalman.lane = currLane;
        kalman.angle = currLane.angle;
        filteredLane = currLane;
        return;
    end
    
    % 预测
    x_pred = kalman.A * kalman.x;
    P_pred = kalman.A * kalman.P * kalman.A' + kalman.Q;
    
    % 更新
    y = z - kalman.H * x_pred;
    S = kalman.H * P_pred * kalman.H' + kalman.R;
    K = P_pred * kalman.H' / S;
    
    kalman.x = x_pred + K * y;
    kalman.P = (eye(2) - K * kalman.H) * P_pred;
    
    % 重建车道线
    x_bottom_filtered = kalman.x(1);
    x_top_filtered = kalman.x(2);
    
    dx = x_bottom_filtered - x_top_filtered;
    dy = imgHeight - roiStart;
    
    filteredLane = currLane;
    filteredLane.slope = dx / dy;
    filteredLane.angle = atand(filteredLane.slope);
    filteredLane.point1 = [x_top_filtered, 0];
    filteredLane.point2 = [x_bottom_filtered, imgHeight - roiStart];
    
    kalman.lane = filteredLane;
    kalman.angle = filteredLane.angle;
end
