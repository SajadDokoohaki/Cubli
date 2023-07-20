% Initialize a new figure
figure

% Loop for 4x4 subplots
for i = 1:16
    % Create a subplot at i-th position with adjusted margins
    subplot('Position', [mod(i-1,4)*0.24+0.05, floor((i-1)/4)*0.24+0.05, 0.2, 0.18])

    % Generate x values
    x = 0:0.01:2*pi;

    % Generate y values for sin wave with i as frequency
    y = sin(i*x);

    % Plot y vs x
    plot(x, y)

    % Set title for each subplot
    title(['sin(', num2str(i), '*x)'])
end

% Add a global title to the figure
suptitle('My Figure Title')
