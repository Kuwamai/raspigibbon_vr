# raspigibbon_vr
Raspberry Pi GibbonをHTC VIVEから動かすROSパッケージ

# Usage
Raspberry Pi Gibbonのシミュレータを起動  
`$ roslaunch raspigibbon_gazebo raspigibbon_with_emptyworld.launch`  
スクリプトを実行  
`$ rosrun rosrun raspigibbon_vr vr_controller.py`  
コントローラの位置姿勢を送信(今回はrosbag)  
`$ rosbag play bagfiles/2018-11-27-03-01-54-trimmed.bag`  
