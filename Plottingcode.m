%% Plotting 2D SLAM solution using GTSAM - Batch and iSAM
figure(1)
plot(x_orig2D,y_orig2D)
grid on
legend("Unoptimized trajectory")
hold on
plot(x_batch2D,y_batch2D)
grid on
legend("Optimized trajectory - batch")

figure(2)
plot(x_orig2D,y_orig2D)
grid on
legend("Unoptimized trajectory")
hold on
plot(x_incr2D,y_incr2D)
grid on
legend("Optimized trajectory - iSAM")

%% Plotting 3D SLAM solution using GTSAM - Batch and iSAM
figure(3)
plot3(x_orig,y_orig,z_orig)
grid on
legend("Unoptimized trajectory")
hold on
plot3(x_batch,y_batch, z_batch)
grid on
legend("Optimized trajectory - batch")

figure(4)
plot3(x_orig,y_orig,z_orig)
grid on
legend("Unoptimized trajectory")
hold on
plot3(x_incr,y_incr, z_incr)
grid on
legend("Optimized trajectory - iSAM")

