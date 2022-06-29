filepath = strcat(pwd,"/Simscape_CAD/");

shank_handle = 2.010001220703125e+02;
thigh_handle = 2.130001220703125e+02;
pulley_handle = 2.960001220703125e+02;
knee_motor_handle = 2.630001220703125e+02;
knee_plate_handle = gcbh;
hip_motor_handle = 1.350001220703125e+02;
hip_plate_handle = 1.370001220703125e+02;

set_param(shank_handle,"ExtGeomFileName",strcat(filepath,"shank.STEP"));
set_param(thigh_handle,"ExtGeomFileName",strcat(filepath,"thigh.STEP"))
set_param(pulley_handle,"ExtGeomFileName",strcat(filepath,"pulley.STEP"))
set_param(knee_motor_handle,"ExtGeomFileName",strcat(filepath,"AK80-6 Motor.STEP"))
set_param(knee_plate_handle,"ExtGeomFileName",strcat(filepath,"hip motor plate.STEP"))
set_param(hip_motor_handle,"ExtGeomFileName",strcat(filepath,"AK80-6 Motor.STEP"))
set_param(hip_plate_handle,"ExtGeomFileName",strcat(filepath,"hip motor plate.STEP"))