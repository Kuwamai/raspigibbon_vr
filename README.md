# raspigibbon_vr

## Description
* Raspberry Pi Gibbonの手先位置をVRコントローラ等で制御できます
* カメラ操作用にアームの手先をパンチルト動作するスクリプトもあります
* 動作してる様子はこんな感じ↓
  * [VRでロボットを操作する_動作確認](https://youtu.be/M2WgAUXNwz8)
  * [VRでロボットを操作する_文字書き編](https://youtu.be/e6eugsj6CXo)
  
## Requirements
* [Raspberry Pi Gibbon](https://products.rt-net.jp/ric/raspberry-pi-gibbon)(操作するロボットアーム)
* Ubuntu16.04
* ROS Kinetic
* [raspigibbon_ros](https://github.com/raspberrypigibbon/raspigibbon_ros)

## Installation
1. [raspigibbon_ros](https://github.com/raspberrypigibbon/raspigibbon_ros)をインストールする
1. 本リポジトリをダウンロード、ビルドする
    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/Kuwamai/raspigibbon_vr.git
    $ cd ~/catkin_ws && catkin_make
    $ source ~/catkin_ws/devel/setup.bash
    ```

## Usage
1. シミュレータ、または実機のRaspberry Pi Gibbonを起動させる
1. 両手、頭部の位置姿勢をPublishさせる  
 例えば下記コマンドを実行し、このようなTopicがPublishされる状態にする
 
    ```
    $ rostopic list -v
    Published topics:
     * /controller_l [geometry_msgs/PoseStamped] 1 publisher
     * /trigger_l [std_msgs/Float64] 1 publisher
     * /controller_r [geometry_msgs/PoseStamped] 1 publisher
     * /trigger_r [std_msgs/Float64] 1 publisher
     * /head [geometry_msgs/PoseStamped] 1 publisher
    ```

1. `/controller_r`と同期し、1台のアームのみを操作する場合は下記コマンド
    ```
    $ roslaunch raspigibbon_vr arm_controller.launch
    ```
1. 複数アームを動作させる場合はそれぞれのアームの下記ファイルを編集し、アームがSubscribeするTopic名を変更する
  [raspigibbon_ros/raspigibbon_bringup/launch/raspigibbon_joint_subscriber.launch](https://github.com/raspberrypigibbon/raspigibbon_ros/blob/kinetic-devel/raspigibbon_bringup/launch/raspigibbon_joint_subscriber.launch#L3)  
  `<group ns="raspigibbon">`→`<group ns="raspigibbon_r">`  
  `<group ns="raspigibbon">`→`<group ns="raspigibbon_l">`  
  `<group ns="raspigibbon">`→`<group ns="raspigibbon_h">`
1. 変更できたら下記コマンドを実行する
    ```
    $ roslaunch raspigibbon_vr arm_controller.launch arm_name:=raspigibbon_r arm_lr:=r
    $ roslaunch raspigibbon_vr arm_controller.launch arm_name:=raspigibbon_l arm_lr:=l
    $ roslaunch raspigibbon_vr pan_tilt_controller.launch arm_name:=raspigibbon_h
    ```

## Bagfiles
* 下記コマンドで動作確認用のrosbagが再生される  
`$ rosbag play bagfiles/2018-11-27-03-01-54-trimmed.bag`  
* HMDの動作確認は45秒後から  
`$ rosbag play -s 45 bagfiles/2018-11-27-03-01-54-trimmed.bag`  

## License
This repository is licensed under the MIT license, see [LICENSE](./LICENSE).
