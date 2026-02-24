close all;clear;clc
%% 数据读取
rgbimg = imread('车道线.png');  % 读取RGB图像数据   
grayImage = rgb2gray(rgbimg); % 转换为灰度图像
% grayImage=double(grayImage);  % 转换数据格式

% ROI区域：底部40%
fullImgHeight = size(grayImage, 1);
roiStart = round(fullImgHeight * 0.6);  % 从60%位置开始，即底部40%
grayImage = grayImage(roiStart:end,:);
imshow(grayImage)
binaryImage = im2bw(grayImage,0.5);  % 设定阈值 将图像二值化
% binaryImage = im2double(grayImage);
% binaryImage = imbinarize(grayImage)
imshow(binaryImage)

%% 使用Sobel算子梯度角度计算
Sx = [-1 0 1; -2 0 2; -1 0 1];
Sy = [1 2 1; 0 0 0; -1 -2 -1];
% 
% for i = 2:size(grayImage,1)
%    for j= 2: size(grayImage,2)
%        A = binaryImage(i-1:i+1, j-1:j+1);
%        G_X = Gx*A'; G_X = sum(diag(G_X));
%        G_Y = Gy*A'; G_Y = sum(diag(G_Y));
%    end
% end

G_x = imfilter(binaryImage, Sx, 'replicate', 'conv'); % 水平梯度
G_y = imfilter(binaryImage, Sy, 'replicate', 'conv'); % 垂直梯度

% --- 梯度幅值与方向 ---
mag = sqrt(G_x.^2 + G_y.^2);             % 梯度幅值
G_x = double(G_x);G_y = double(G_y);
theta = atan2d(G_y, G_x);                % 方向（单位：度）
theta(theta<0) = theta(theta<0) + 180; % 转为 [0,180) 区间（便于后续计算）

% --- 归一化幅值用于显示 ---
mag_norm = mat2gray(mag);  % 归一化到 [0,1]

% --- 自适应/手动阈值二值化边缘 ---
% 方案 A：Otsu 自动阈值分割二值化
th_otsu = graythresh(mag_norm);
edge_otsu = mag_norm >= th_otsu;

% 方案 B：手动阈值分割二值化
th_manual = 0.2;                    % 可调整0~1
edge_manual = mag_norm >= th_manual;

% 方案 C：使用 MATLAB 自带 edge 函数 (Sobel)
edge_builtin = edge(binaryImage, 'Sobel'); % MATLAB 默认会基于梯度幅值计算并自动阈值

% --- 可视化结果 ---
figure('Name','Sobel Edge Detection','NumberTitle','off','Position',[100 100 1200 600]);

subplot(2,4,1);
imshow(binaryImage); title('原图 (灰度)');

subplot(2,4,2);
imshow(G_x,[]); colormap(gca,jet); colorbar;
title('G_x (水平梯度)');

subplot(2,4,3);
imshow(G_y,[]); colormap(gca,jet); colorbar;
title('G_y (垂直梯度)');

subplot(2,4,4);
imshow(mag_norm,[]); title('梯度幅值(归一化)');

subplot(2,4,5);
imshow(edge_otsu); title(['边缘(Otsu 阈值 t=' num2str(th_otsu,3) ')']);

subplot(2,4,6);
imshow(edge_manual); title(['边缘(手动阈值 t=' num2str(th_manual) ')']);

subplot(2,4,7);
imshow(edge_builtin); title('自带 edge(...,"Sobel")');

% 同一张图上叠加边缘结果用于比较：
subplot(2,4,8);
imshow(binaryImage); hold on;
h = imshow(cat(3, ones(size(edge_builtin)), zeros(size(edge_builtin)), zeros(size(edge_builtin))));
set(h, 'AlphaData', 0.4*edge_builtin); % 红色半透明叠加检测到的边缘
title('叠加 Sobel 边缘 (红色覆盖)');
hold off;

% --- 输出一些信息 ---
fprintf('Otsu 阈值 (on norm mag) = %.4f\n', th_otsu);
fprintf('手动阈值 = %.4f\n\n', th_manual);



%%  霍夫变换检测直线

[H, theta, rho] = hough(edge_otsu);
peaks = houghpeaks(H, 10, 'Threshold', 0.3 * max(H(:)));
lines = houghlines(edge_otsu, theta, rho, peaks, 'FillGap', 20, 'MinLength', 30);

figure; imshow(rgbimg); hold on;
title('检测到的直线结果');

max_len = 0;
for k =1:10   % 提取10条直线，选择属于车道线的直线
    xy = [lines(k).point1; lines(k).point2];
    
    % 绘制线段
    plot(xy(:,1), xy(:,2)+roiStart, 'LineWidth', 3, 'Color', 'red');
    
    % 标记端点
    plot(xy(1,1), xy(1,2)+roiStart, 'rx', 'LineWidth', 2);
    plot(xy(2,1), xy(2,2)+roiStart, 'gx', 'LineWidth', 2);
    
    % 寻找最长线段（可选）
    len = norm(lines(k).point1 - lines(k).point2);
    if len > max_len
        max_len = len;
        longest_line = xy;
    end
end

%% 自动剔除无效直线 - 优化版本
% 问题：图中有多个车道时，检测到的直线太多
% 优化点：1. 使用位置信息优先选择中间车道线 2. 支持多车道线的检测
% 
% 筛选策略：
% 1. 只保留两条车道线（左边一条+右边一条）
% 2. 优先选择靠近图像中间的
% 3. 综合长度和角度进行评分

numLines = length(lines);
if numLines > 0
    % ========== 获取图像信息 ==========
    imgHeight = size(binaryImage, 1);
    imgWidth = size(binaryImage, 2);
    imgCenterX = imgWidth / 2;
    
    % ========== 提取直线信息 ==========
    lineInfo = struct('point1', {}, 'point2', {}, 'slope', {}, 'angle', {}, ...
                      'midpoint', {}, 'length', {}, 'distanceToCenter', {}, ...
                      'score', {}, 'side', {});
    
    for k = 1:numLines
        p1 = lines(k).point1;
        p2 = lines(k).point2;
        
        % 计算斜率和角度
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
        distToCenter = abs(midpoint(1) - imgCenterX);
        
        lineInfo(k).point1 = p1;
        lineInfo(k).point2 = p2;
        lineInfo(k).slope = slope;
        lineInfo(k).angle = angle;
        lineInfo(k).midpoint = midpoint;
        lineInfo(k).length = len;
        lineInfo(k).distanceToCenter = distToCenter;
        
        % 判断左右，标记为'left'、'right'、'unknown'
        if slope < -0.3 && slope > -5
            lineInfo(k).side = 'left';
        elseif slope > 0.3 && slope < 5
            lineInfo(k).side = 'right';
        else
            lineInfo(k).side = 'unknown';
        end
    end
    
    % ========== 第1步：初步过滤，排除不合理角度 ==========
    filteredLines = [];
    for k = 1:numLines
        angle = lineInfo(k).angle;
        len = lineInfo(k).length;
        side = lineInfo(k).side;
        
        % 角度限制：车道线倾斜角在 20-70 度，排除水平和垂直
        angleValid = (abs(angle) >= 20 && abs(angle) <= 70);
        
        % 长度不能太短
        lenValid = len > 40;
        
        % 必须能判断左右
        sideValid = ~strcmp(side, 'unknown');
        
        if angleValid && lenValid && sideValid
            filteredLines = [filteredLines; lineInfo(k)];
        end
    end
    
    fprintf('第1步 初步过滤后剩余 %d 条直线\n', length(filteredLines));
    
    % ========== 第2步：按左右分类 ==========
    leftLines = [];
    rightLines = [];
    
    for k = 1:length(filteredLines)
        if strcmp(filteredLines(k).side, 'left')
            leftLines = [leftLines; filteredLines(k)];
        else
            rightLines = [rightLines; filteredLines(k)];
        end
    end
    
    % ========== 第3步：选择两条车道线（左、右）==========
    % 策略：
    % - 左边车道线：在所有左斜线中，选择评分最高的一条
    % - 右边车道线：在所有右斜线中，选择评分最高的一条
    
    selectedLeft = [];      % 左边车道线
    selectedRight = [];     % 右边车道线
    
    % 计算所有候选线的评分
    for k = 1:length(filteredLines)
        len = filteredLines(k).length;
        dist = filteredLines(k).distanceToCenter;
        angle = filteredLines(k).angle;
        % 评分公式：长度权重高，距离中心近的加分
        filteredLines(k).score = len * 2 - dist * 0.5 + abs(angle) * 0.3;
    end
    
    % 选择左边车道线（左斜线中评分最高的）
    if ~isempty(leftLines)
        bestScore = -inf;
        for k = 1:length(leftLines)
            if leftLines(k).score > bestScore
                bestScore = leftLines(k).score;
                selectedLeft = leftLines(k);
            end
        end
    end
    
    % 选择右边车道线（右斜线中评分最高的）
    if ~isempty(rightLines)
        bestScore = -inf;
        for k = 1:length(rightLines)
            if rightLines(k).score > bestScore
                bestScore = rightLines(k).score;
                selectedRight = rightLines(k);
            end
        end
    end
    
    % ========== 第4步：备选方案 ==========
    % 如果某条车道线缺失，尝试补充
    if isempty(selectedLeft) && ~isempty(leftLines)
        selectedLeft = leftLines(1);
    end
    if isempty(selectedRight) && ~isempty(rightLines)
        selectedRight = rightLines(1);
    end
    
    fprintf('最终选中: 左=%d条, 右=%d条\n', ...
        ~isempty(selectedLeft), ~isempty(selectedRight));
    
    % ========== 第5步：绘制结果 ==========
    figure('Name','优化后的车道线','NumberTitle','off');
    imshow(rgbimg); hold on;
    title('优化后剔除的车道线检测结果');
    
    % 显示所有候选直线（灰色虚线）
    for k = 1:length(filteredLines)
        xy = [filteredLines(k).point1; filteredLines(k).point2];
        plot(xy(:,1), xy(:,2)+roiStart, '--', 'LineWidth', 1, 'Color', [0.5 0.5 0.5 0.3]);
    end
    
    % 显示选中的左车道线（蓝色）
    if ~isempty(selectedLeft)
        xy = [selectedLeft.point1; selectedLeft.point2];
        plot(xy(:,1), xy(:,2)+roiStart, 'LineWidth', 5, 'Color', 'blue');
        plot(xy(1,1), xy(1,2)+roiStart, 'bx', 'LineWidth', 2, 'MarkerSize', 12);
        plot(xy(2,1), xy(2,2)+roiStart, 'bo', 'LineWidth', 2, 'MarkerSize', 12);
        fprintf('选中左车道线: 角度=%.1f°, 长度=%.1f, 距中心=%.1f\n', ...
            selectedLeft.angle, selectedLeft.length, selectedLeft.distanceToCenter);
    end
    
    % 显示选中的右车道线（绿色）
    if ~isempty(selectedRight)
        xy = [selectedRight.point1; selectedRight.point2];
        plot(xy(:,1), xy(:,2)+roiStart, 'LineWidth', 5, 'Color', 'green');
        plot(xy(1,1), xy(1,2)+roiStart, 'gx', 'LineWidth', 2, 'MarkerSize', 12);
        plot(xy(2,1), xy(2,2)+roiStart, 'go', 'LineWidth', 2, 'MarkerSize', 12);
        fprintf('选中右车道线: 角度=%.1f°, 长度=%.1f, 距中心=%.1f\n', ...
            selectedRight.angle, selectedRight.length, selectedRight.distanceToCenter);
    end
    
    % 显示图像中心线
    plot([imgCenterX, imgCenterX], [roiStart, size(rgbimg,1)], 'y--', 'LineWidth', 1);
    
    legend('候选直线', '左车道线', '右车道线', '图像中心');
    hold off;
    
    % ========== 第6步：延长到图像边界 ==========
    figure('Name','延长到图像边界的车道线','NumberTitle','off');
    imshow(rgbimg); hold on;
    title('延长到图像边界的车道线（优化版）');
    
    imgHeight_full = size(rgbimg, 1);
    imgWidth_full = size(rgbimg, 2);
    
    % 延长左车道线（蓝色）
    if ~isempty(selectedLeft)
        p1 = selectedLeft.point1;
        p2 = selectedLeft.point2;
        p1_full = [p1(1), p1(2)+roiStart];
        p2_full = [p2(1), p2(2)+roiStart];
        
        slope = selectedLeft.slope;
        if abs(slope) > 0.01 && abs(slope) < 100
            % 延长到底部
            y_bottom = imgHeight_full;
            x_bottom = p2_full(1) + (y_bottom - p2_full(2)) / slope;
            
            % 延长ROI顶部
            y_top = roiStart;
            x_top = p1_full(1) + (y_top - p1_full(2)) / slope;
            
            plot([x_top, x_bottom], [y_top, y_bottom], 'LineWidth', 5, 'Color', 'blue');
        end
    end
    
    % 延长右车道线（绿色）
    if ~isempty(selectedRight)
        p1 = selectedRight.point1;
        p2 = selectedRight.point2;
        p1_full = [p1(1), p1(2)+roiStart];
        p2_full = [p2(1), p2(2)+roiStart];
        
        slope = selectedRight.slope;
        if abs(slope) > 0.01 && abs(slope) < 100
            y_bottom = imgHeight_full;
            x_bottom = p2_full(1) + (y_bottom - p2_full(2)) / slope;
            
            y_top = roiStart;
            x_top = p1_full(1) + (y_top - p1_full(2)) / slope;
            
            plot([x_top, x_bottom], [y_top, y_bottom], 'LineWidth', 5, 'Color', 'green');
        end
    end
    
    hold off;
else
    fprintf('未检测到任何直线\n');
end
