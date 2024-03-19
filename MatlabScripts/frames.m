% Define the origin
origin = [0; 0; 0];

% Define the axes unit vectors
x_axis = [1; 0; 0];
y_axis = [0; 1; 0];
z_axis = [0; 0; 1];

% Plot the axes in 3D space
figure;
hold on;
quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), 'r', 'LineWidth', 2);
quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), 'g', 'LineWidth', 2);
quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), 'b', 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Original Frame');
axis equal;
grid on;
view(3);

% Transformation matrix (example)
transformation_matrix =[0 0 1 1; 
                        0 2 0 1;
                        -1 0 0 1;
                        0 0 0 2];

% Apply transformation to the axes
transformed_x_axis = transformation_matrix * [x_axis; 1];
transformed_y_axis = transformation_matrix * [y_axis; 1];
transformed_z_axis = transformation_matrix * [z_axis; 1];

% Plot the transformed axes
draw_frame(origin, transformed_x_axis(1:3), transformed_y_axis(1:3), transformed_z_axis(1:3), 'Transformed Frame');

function draw_frame(origin, x_axis, y_axis, z_axis, title_text)
    % Plot the axes in 3D space
    figure;
    hold on;
    quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), 'g', 'LineWidth', 2);
    quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), 'b', 'LineWidth', 2);
    quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), 'r', 'LineWidth', 2);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title(title_text);
    axis equal;
    grid on;
    view(3);
end
