% ==========================================
% 1. INPUT YOUR VALUES HERE
% ==========================================
% From Transform1 (Gear)
t1 = smiData.RigidTransform(31).translation;        % Replace with Transform1 Cartesian Translation [x, y, z]
axis1 = smiData.RigidTransform(31).axis;           % Replace with Transform1 Arbitrary Axis
angle1 = smiData.RigidTransform(31).angle;        % Replace with Transform1 Angle (Ensure it's in radians!)

% From Transform7 (Gripper)
t7 = smiData.RigidTransform(27).translation;     % Replace with Transform7 Cartesian Translation [x, y, z]
axis7 = smiData.RigidTransform(27).axis;           % Replace with Transform7 Arbitrary Axis
angle7 = smiData.RigidTransform(27).angle;        % Replace with Transform7 Angle (Ensure it's in radians!)

% ==========================================
% 2. CALCULATION
% ==========================================
% Convert axis-angle to Rotation Matrices
% Note: axang2rotm takes the input as [x, y, z, theta]
R1 = axang2rotm([axis1, angle1]);
R7 = axang2rotm([axis7, angle7]);

% Build Homogeneous Matrices
T1 = trvec2tform(t1) * rotm2tform(R1);
T7 = trvec2tform(t7) * rotm2tform(R7);

% Calculate Relative Transformation matrix
T_rel = T1 \ T7;  % This is mathematically equivalent to inv(T1) * T7

% Extract the new Translation and Rotation
t_rel = tform2trvec(T_rel);           % Cartesian translation
axang_rel = tform2axang(T_rel);       % Returns [x, y, z, angle]

% ==========================================
% 3. RESULTS FOR YOUR NEW SIMSCAPE BLOCK
% ==========================================
fprintf('\n--- INPUT THESE INTO YOUR NEW RIGID TRANSFORM BLOCK ---\n');
fprintf('Method: Cartesian / Standard Axis\n');
fprintf('Translation [x, y, z]: [%.4f, %.4f, %.4f]\n', t_rel(1), t_rel(2), t_rel(3));
fprintf('Rotation Axis [x, y, z]: [%.4f, %.4f, %.4f]\n', axang_rel(1), axang_rel(2), axang_rel(3));
fprintf('Rotation Angle (rad): %.4f\n', axang_rel(4));
fprintf('Rotation Angle (deg): %.4f\n', rad2deg(axang_rel(4)));