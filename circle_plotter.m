function circle_plotter()
    % Define parameters for the X sine wave
    amplitudeX = 1;
    biasX = 2.33;
    frequencyX = 0.4;
    phaseShiftX = 0;

    % Define parameters for the Y sine wave
    amplitudeY = 1;
    biasY = 2.55;
    frequencyY = 0.4;
    phaseShiftY = 1.57; % Approximately pi/2, for orthogonal sine waves

    % Generate time vector
    t = linspace(0, 2*pi, 1000); % Cover one full cycle with high resolution

    % Calculate X and Y coordinates
    x = amplitudeX * sin(2 * pi * frequencyX * t + phaseShiftX) + biasX;
    y = amplitudeY * sin(2 * pi * frequencyY * t + phaseShiftY) + biasY;
    z = ones(size(x)); % Z coordinate at 1m altitude

    % Plotting the circle in 3D
    figure; % Open a new figure window
    plot3(x, y, z, 'LineWidth', 3);
    grid on; % Enable grid for better visualization
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    title(['Circular trajectory using Linear MPC' ...
        '(25 sec)']);
end
