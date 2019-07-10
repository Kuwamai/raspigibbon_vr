# raspigibbon_vr
Raspberry Pi GibbonをHTC VIVEから動かすROSパッケージ

# Usage
1. Raspberry Pi Gibbonのシミュレータを起動  
`$ roslaunch raspigibbon_gazebo raspigibbon_with_emptyworld.launch`  
rvizの場合は下記コマンド  
`$ roslaunch raspigibbon_bringup rviz_joint_subscriber.launch`  
1. スクリプトを実行  
`$ rosrun rosrun raspigibbon_vr vr_controller.py`  
1. コントローラの位置姿勢を送信(今回はrosbag)  
`$ rosbag play bagfiles/2018-11-27-03-01-54-trimmed.bag`  
