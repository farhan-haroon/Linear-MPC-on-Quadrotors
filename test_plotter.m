function test_plotter(rosbagPath)
    % Plot the first Lissajous curve based on sine wave parameters
    t = linspace(0, 2*pi, 1000); % Time vector for plotting

    % Sine wave parameters for X
    amplitudeX = 1;
    biasX = 2.33;
    frequencyX = 0.6;
    phaseShiftX = 0;

    % Sine wave parameters for Y
    amplitudeY = 1;
    biasY = 2.55;
    frequencyY = 0.2;
    phaseShiftY = 1.57;

    % Calculate the X and Y coordinates
    x = amplitudeX * sin(2 * pi * frequencyX * t + phaseShiftX) + biasX;
    y = amplitudeY * sin(2 * pi * frequencyY * t + phaseShiftY) + biasY;
    z = ones(size(x)); % Fixed Z value

    % Plotting the first Lissajous curve
    figure;
    plot3(x, y, z, 'LineWidth', 3);
    zlim([0 1.5])
    hold on; % Keep the figure for the next plot
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    title('Lissajous curve using Linear MPC');

    % Load the ROS bag file
    bag = rosbag(rosbagPath);
    
    % Select the topic and filter by the first 60 seconds
    startTime = bag.StartTime;
    endTime = startTime + 35; % First 60 seconds
    topicSelect = select(bag, 'Topic', '/mavros/vision_pose/pose', 'Time', [startTime endTime]);

    % Read the messages
    msgs = readMessages(topicSelect, 'DataFormat', 'struct');

    % Extract position data
    x_ros = cellfun(@(m) double(m.Pose.Position.X), msgs);
    y_ros = cellfun(@(m) double(m.Pose.Position.Y), msgs);
    z_ros = ones(size(x_ros)); % Fixed Z value for the ROS data

    % Plot the ROS bag data as another Lissajous curve with a continuous red line
    for i = 1:numel(x_ros)
        % Plot a line segment at a time
        if i > 1
            plot3(x_ros(i-1:i), y_ros(i-1:i), z_ros(i-1:i), 'r', 'LineWidth', 2.5);
        end

        % Pause to slow down the graph plotting
        pause(0.1); % Adjust the argument based on your desired delay in seconds
    end

    % Enhancing the plot
    legend('Reference trajectory', 'Actual trajectory')
    hold off;
end
