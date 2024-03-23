function rosbag_circle_plotter(rosbagPath)
    % First, plot the circle from sine waves
    circle_plotter(); % This will create the figure and plot the first circle

    hold on; % Hold on to plot the next circle on the same figure

    % Load the ROS bag file
    bag = rosbag(rosbagPath);

    % Get the start time of the bag
    startTime = bag.StartTime;
    
    % Calculate the end time for the 20-second interval
    endTime = startTime + 25;

    % Select the topic with position data within the first 20 seconds
    topicSelect = select(bag, 'Topic', '/mavros/vision_pose/pose', 'Time', [startTime, endTime]);

    % Read the messages from the selected topic
    msgs = readMessages(topicSelect, 'DataFormat', 'struct');

    % Extract the X, Y, Z positions
    x_ros = cellfun(@(m) double(m.Pose.Position.X), msgs);
    y_ros = cellfun(@(m) double(m.Pose.Position.Y), msgs);
    z_ros = cellfun(@(m) double(m.Pose.Position.Z), msgs);

    % Plot the ROS bag data
    plot3(x_ros, y_ros, z_ros, 'r', 'LineWidth', 2.5);

    legend('Reference trajectory', 'Actual trajectory');
    zlim([0 1.5]);

    hold off;
end
