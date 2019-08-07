# raspigibbon_vr
Raspberry Pi GibbonをHTC VIVEから動かすROSパッケージ

# Usage
## シミュレータの起動
* GAZEBOの場合
  * `$ roslaunch raspigibbon_gazebo raspigibbon_with_emptyworld.launch`  
* rvizの場合
  * `$ roslaunch raspigibbon_bringup rviz_joint_subscriber.launch`  

## コントローラとアームの位置姿勢を同期
`$ rosrun raspigibbon_vr vr_controller.py`

## HMDとカメラの姿勢を同期
`$ rosrun raspigibbon_vr pan_tilt_controller.py`

## rosbagの再生
`$ rosbag play bagfiles/2018-11-27-03-01-54-trimmed.bag`  
* HMDの動作確認は40秒後から  
`$ rosbag play -s 45 bagfiles/2018-11-27-03-01-54-trimmed.bag`  
