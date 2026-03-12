% ==========================================
% 1. INPUT YOUR VALUES HERE
% ==========================================
% From Transform1 (Gear)
n_parent = 24;
t_parent = smiData.RigidTransform(n_parent).translation;        % Replace with Transform1 Cartesian Translation [x, y, z]
axis_parent = smiData.RigidTransform(n_parent).axis;           % Replace with Transform1 Arbitrary Axis
angle_parent = smiData.RigidTransform(n_parent).angle;        % Replace with Transform1 Angle (Ensure it's in radians!)

% From Transform7 (Gripper)
n_child = 50;
t_child = smiData.RigidTransform(n_child).translation;     % Replace with Transform7 Cartesian Translation [x, y, z]
axis_child = smiData.RigidTransform(n_child).axis;           % Replace with Transform7 Arbitrary Axis
angle_child = smiData.RigidTransform(n_child).angle;        % Replace with Transform7 Angle (Ensure it's in radians!)

% ==========================================
% 2. CALCULATION
% ==========================================
% Convert axis-angle to Rotation Matrices
% Note: axang2rotm takes the input as [x, y, z, theta]
R1 = axang2rotm([axis_parent, angle_parent]);
R7 = axang2rotm([axis_child, angle_child]);

% Build Homogeneous Matrices
t_parent = trvec2tform(t_parent) * rotm2tform(R1);
t_child = trvec2tform(t_child) * rotm2tform(R7);

% Calculate Relative Transformation matrix
T_rel = t_parent \ t_child;  % This is mathematically equivalent to inv(t_parent) * t_child

% Extract the new Translation and Rotation
t_rel = tform2trvec(T_rel);           % Cartesian translation
axang_rel = tform2axang(T_rel);       % Returns [x, y, z, angle]

% ==========================================
% 3. RESULTS FOR YOUR NEW SIMSCAPE BLOCK
% ==========================================
index = 10;
sim_relative(index).axis = [axang_rel(1), axang_rel(2), axang_rel(3)];
sim_relative(index).angle = axang_rel(4);
sim_relative(index).translation = [t_rel(1), t_rel(2), t_rel(3)];
fprintf('\n--- INPUT THESE INTO YOUR NEW RIGID TRANSFORM BLOCK ---\n');
fprintf('Method: Cartesian / Standard Axis\n');
fprintf('Translation [x, y, z]: [%.4f, %.4f, %.4f]\n', t_rel(1), t_rel(2), t_rel(3));
fprintf('Rotation Axis [x, y, z]: [%.4f, %.4f, %.4f]\n', axang_rel(1), axang_rel(2), axang_rel(3));
fprintf('Rotation Angle (rad): %.4f\n', axang_rel(4));
fprintf('Rotation Angle (deg): %.4f\n', rad2deg(axang_rel(4)));