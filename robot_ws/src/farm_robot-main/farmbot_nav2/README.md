# farmbot_nav2 

##### ဒါတွေ install လုပ်ထားပါ။
```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-twist-mux ros-humble-nav2*     
sudo apt install ros-humble-robot-localization
sudo apt install -y ros-humble-slam-toolbox
```
#### for develop

```
LaunchConfiguration() ကြောင့် yaml file များ overwrite ဖြစ်နိုင်သည်။
nav2 node အချို့တွင် paramfile ကို သက်ဆိုင်ရာ node သို့ input သွင်းပီး သုံးရတာရှိသလို input မသွင်းလဲ သုံးလို့ရနေနိုင်ပါသည်။ 
```
```
## Q & A
##### simulation distance 1 meter နဲ့ rviz 1m, sim rotate တပါတ်နဲ့ rviz rotate တပါတ်မတူရင် ဘယ်လိုလုပ်ရမလဲ။ ( fixed )
##### gazebo empty world တွင် 360 degree နဲ့ rviz 360 စစ်ပါ။ မတူရင် ros2_control ထဲမှာ ပြင်ဆင်ပါ။ rotate လဲစစ်ပါ။
fixed rom2109_description/config/mycontrollers.yaml , wheel_radius: 0.04355 instead of 0.035
wheel_separation: 0.21 instead of 0.1966, fixed OK
```
ros2 launch rom2109_gazebo rom2109_sim_ros2_control_empty_world.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/diff_cont/cmd_vel_unstamped
```
##### controller တခါတလေနောက်ကျ၊ တက်မလာရင် ဘယ်လိုလုပ်ရမလဲ? ( tempory fixed )
##### delay နဲ့ရေးပါ။ လတ်တလောတော့ gazebo, robot spawn သက်သက် controller သက်သက် ခွဲရေးတယ်။
```
ros2 launch rom2109_gazebo delayed_rom2109_sim_ros2_control.launch.py
```
အပေါ်က launch ဖိုင်မျာဆိုရင် controller manager နဲ့ သက်ဆိုင်ရာ controller spawner များကို launch ထားပေမဲ့ တခါတလေ spawner များ တက်မလာတာဖြစ်တတ်တယ်။ ဒါကြောင့်  controller spawner များကို launch ဖိုင်အသစ်ဖြင့် ခွဲထားပြီး အောက်ပါအတိုင်း launch ပါ။ launch ဖိုင် မခွဲချင်ရင်တော့  Delay ကို အသေးစိတ်ထပ်ရေးဖို့လိုပါတယ်။
```
ros2 launch rom2109_gazebo rom2109_sim_ros2_control.launch.py
# gazebo ပွင့်ပြီး robot spawn ဖြစ်မှာ အောက်ပါ controller ကို run ပါ။
ros2 launch rom2109_gazebo controller_spawner.launch.py
```
##### Q: Robot  ရဲ့ လက်ရှိ Pose ကို ဘယ်လိုသိနိုင်မလဲ?
##### A: ros2 topic echo /amcl_pose
##### Q: nav2_params.yaml ဖိုင် ဘယ်လိုပြောင်းရမလဲ?
```
sed -i 's/nav2_params.yaml/nav2_params_collision.yaml/g' /path/to/hw_localization_init_pose_launch.py
sed -i 's/nav2_params.yaml/nav2_params_collision.yaml/g' /path/to/hw_localization_launch.py
```
## -------------------------------------------------------------------------------------------
## ပြင်ရန်ကျန်
##### Groot နဲ့ချိတ်ရန်
##### nav2_params.yaml မှာ robot_radius, cost_scaling_factor, inflation_radius တို့ကိုပြင်ပါ။
##### farmbot_nav2 မှ init_robot_pose.cpp မှာ publisher ရဲ့ get subscription count ရယူပြီး code ပြင်ရေးရန်။





<a href="https://github.com/ROM-robotics/rom2109_simulation">မူလစာမျက်နှာ </a>