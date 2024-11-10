Start：（开机自启）

    rosrun robot_nav_pkg command_service.py 
    rosrun robot_nav_pkg robot_state_machine.py

map：

    build_map:(cartographer_mapping.launch)

            rosservice call /robotCmd command:"mapping"

    save_map: 

            rosservice call /robotCmd command:"save_map"

navigation:

    navigation:(start_naviagtion.py)

           rosservice call /robotCmd command:"navigation"
    
    <!-- 任务文件创建与删除  -->
     
    <!-- 充电charging与初始位置initial任务文件是自动代替，不需要删除和新建 -->
   
     taskMngmt:(taskMngmt.py)

               rosservice call /taskMngmt command:"delete_patrol"
           
               rosservice call /taskMngmt command:"delete_emergency"
           
               rosservice call /taskMngmt command:"new_patrol"
           
               rosservice call /taskMngmt command:"new_emergency"
    
    save_Point:(savePoint.py)
            
               rosservice call /savePoint command:"save_charging"
            
               rosservice call /savePoint command:"save_emergency"
           
               rosservice call /savePoint command:"save_initial"
            
               rosservice call /savePoint command:"save_charging"
   
     <!-- 执行任务命令 -->
   
     Partolling(巡检):(start_patrolling.py)  
           
               rosservice call /robotCmd command:"patrolling"
       
               <!-- 该文件中有多点导航的暂停取消恢复程序，与单点导航暂停取消恢复逻辑不同，但话题名称相同，故不影响使用 -->
    
    Emergency(应急):  start_emergency.py

               rosservice call /emergency command:"num" 
                <!-- 根据 Num 不同，去往不同的应急点 -->

    Charging(充电):   start_charging.py
           
               rosservice call /robotCmd command:"charging"
    
    Initial(初始位置): start_initial.py
         
               rosservice call /robotCmd command:"initial"
    
    Pause(暂停取消恢复): navFunc.py
          
               rosservice call /navFunc command:"pause" / "resume" / "cancel"
   
                <!-- 巡检任务单独采用一套暂停恢复取消逻辑 -->
    
   display_marker.py 

            巡检点、应急点、充电点、初始位置点的点位marker显示
    
   dynamic_configure.py 

            话题名称/dynamic_reconfigure，动态调整max_vel_x,max_vel_x_backwards,max_vel_theta,acc_lim_x,acc_lim_theta参数 

   pointcloud_to_laserscan 

            将3D多线激光雷达扫描到的点云信息压缩到一个2D平面内，对于障碍物与平面之间设置高度差，将障碍物与平面点云数据与区分开来，保留符合条件的坐标点，只保留X,Y坐标，投影到2D平面中，以此来识别到低矮障碍物

遇到问题：
    
        1、动态障碍物存留残影：修改costmap_common_params.yaml文件中的obstacle_range与raytrace_range参数，修改navigation/costmap 2dplugins/obstacle layer.cpp中laserScanValidInfCallback函数的激光雷达返回值的判定条件
    
        2、TEB规划转圈：将前向前进的权重weight_kinematics_forward_drive调高
    
        3、robot进不去狭窄通道：减小障碍物膨胀层，降低转弯半径

