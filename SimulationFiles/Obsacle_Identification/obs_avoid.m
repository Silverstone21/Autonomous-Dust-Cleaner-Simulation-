% Object Avoidance Robot Simulation with Obstacles Prototype

clc
close all
clear

% Parameters
startPoint = [1, 1];       % Starting point of the robot [row, col]
endPoint = [9.5, 9.5];     % Destination point [row, col]
numSteps = 100;            % Number of simulation steps
stepSize = 0.5;            % Reduced size of each simulation step for slower speed
obstacleSpeed = 0.2;       % Speed of the moving obstacle
clearanceRange = 0.3;      % Clearance range around the robot

% Create a binary occupancy grid with static obstacles
occupancyGrid = zeros(10, 10);
occupancyGrid(6, 4:7) = 1;  % Define static obstacles on the grid
occupancyGrid(8, 6:10) = 1;
occupancyGrid(4, 3:5) = 1;  % Define static obstacles on the grid

% Initialize figure
figure;
hold on;
title('Object Avoidance Robot Simulation with Randomly Moving Obstacle');
xlabel('Column Index');
ylabel('Row Index');
axis([0 10 0 10]);
grid on;

% Initialize random position and motion vector for the moving obstacle
obstaclePosition = rand(1, 2) .* [8, 8] + [1, 1];  % Random starting position
obstacleMotionVector = rand(1, 2) * 2 - 1;        % Random motion vector

% Simulation loop
for step = 1:numSteps
    % Display robot
    plot(startPoint(2), startPoint(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    pause(0.1);  % Pause to visualize the movement
    
    % Compute the direction vector towards the destination
    directionVector = endPoint - startPoint;
    
    % Normalize the direction vector to have unit magnitude
    directionVector = directionVector / norm(directionVector);
    
    % Move the robot towards the destination
    startPoint = startPoint + stepSize * directionVector;
    
    % Check for collision with the boundaries
    if startPoint(1) > 10 || startPoint(2) > 10 || startPoint(1) < 1 || startPoint(2) < 1
        disp('Collision with boundaries! Stopping simulation.');
        break;
    end
    
    % Check for collision with static obstacles
    if occupancyGrid(round(startPoint(1)), round(startPoint(2))) == 1
        disp('Collision with static obstacle! Avoiding obstacle.');
        % For simplicity, move the robot to the left to avoid the obstacle
        startPoint = startPoint - [0, stepSize];
    end
    
    % Update the position of the randomly moving obstacle
    obstaclePosition = obstaclePosition + obstacleSpeed * obstacleMotionVector;
    
    % Check for collision with the moving obstacle
    distanceToObstacle = norm(startPoint - obstaclePosition);
    
    if distanceToObstacle < clearanceRange
        disp('Moving obstacle within clearance range! Waiting for clearance.');
        
        % Wait until the moving obstacle is outside the clearance range
        while distanceToObstacle < clearanceRange
            obstaclePosition = obstaclePosition + obstacleSpeed * obstacleMotionVector;
            distanceToObstacle = norm(startPoint - obstaclePosition);
            pause(0.1);
        end
    end
    
    % Clear previous robot position
    clf;
    hold on;
    title('Object Avoidance Robot Simulation with Static & Randomly Moving Obstacle');
    xlabel('Column Index');
    ylabel('Row Index');
    axis([0 10 0 10]);
    grid on;
    
    % Display static obstacles on the grid
    [row_obs, col_obs] = find(occupancyGrid == 1);
    plot(col_obs, row_obs, 'bs', 'MarkerSize', 10, 'LineWidth', 2);

    % Display the moving obstacle
    plot(obstaclePosition(2), obstaclePosition(1), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Display the destination
    plot(endPoint(2), endPoint(1), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Check if the robot has reached the destination
    if norm(startPoint - endPoint) < stepSize
        disp('Robot reached the destination! Stopping simulation.');
        break;
    end
end
