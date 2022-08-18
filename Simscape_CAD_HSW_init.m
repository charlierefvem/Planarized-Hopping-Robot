filepath = strcat(pwd,"/Simscape_CAD/");

shank_path = "Simscape_CAD_HSW/Hopping Robot Model/Hopping Robot Mechanics/Shank Link";
thigh_path = "Simscape_CAD_HSW/Hopping Robot Model/Hopping Robot Mechanics/Thigh Link";
pulley_path = "Simscape_CAD_HSW/Hopping Robot Model/Pulley";
knee_motor_path = "Simscape_CAD_HSW/Hopping Robot Model/Knee Motor AK80-6";
knee_plate_path = "Simscape_CAD_HSW/Hopping Robot Model/Knee Motor Plate"
hip_motor_path = "Simscape_CAD_HSW/Hopping Robot Model/Hip Motor AK80-6";
hip_plate_path = "Simscape_CAD_HSW/Hopping Robot Model/Hip Plate";

set_param(shank_path,"ExtGeomFileName",strcat(filepath,"shank.STEP"));
set_param(thigh_path,"ExtGeomFileName",strcat(filepath,"thigh.STEP"));
set_param(pulley_path,"ExtGeomFileName",strcat(filepath,"pulley.STEP"));
set_param(knee_motor_path,"ExtGeomFileName",strcat(filepath,"AK80-6 Motor.STEP"));
set_param(knee_plate_path,"ExtGeomFileName",strcat(filepath,"hip motor plate.STEP"));
set_param(hip_motor_path,"ExtGeomFileName",strcat(filepath,"AK80-6 Motor.STEP"));
set_param(hip_plate_path,"ExtGeomFileName",strcat(filepath,"hip motor plate.STEP"));