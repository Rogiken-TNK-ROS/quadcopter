# quadcopter
開発にあたって規約は以下に従う(2020/01/10)  
[議事録:第1回2019-08-21](https://wrs2020.esa.io/posts/5)より  

git
- master
    - hotfix/fuga
    - develop　 -> master へのマージは絶対レビュー通過したコードしか入れない
- develop
    - feature/hoge
    - feature/hoge -> develop　は  
    Build, 実行ができていれば各自自由にPR->マージして良い(gitlabだとmerge request?)

## quadcopterリポジトリ
```sh
cd ~/<catkin_ws>/src
git clone https://gitlab.com/rogiken-tnk-ros/quadcopter.git
cp choreonoid/src/MulticopterPlugin/exportdecl.h ../devel/include/choreonoid-1.8/cnoid/src/MulticopterPlugin
```

## プロジェクトの実行
```sh
cd ~/<catkin_ws>/src/quadcopter/cnoid
roscore
choreonoid QuadcopterJoystickWRS.cnoid
```
DUALSHOCK 4 (PS4コントローラ) での操作を想定.

- Xボタンでロータ始動
- 左スティック上下で上昇/下降
- 左スティック左右でヨー軸
- 右スティックで移動
- 右スティック放すとホバリング
- 十字キー上下でカメラのピッチ軸
- 十字キー 右 カメラ2ズームイン
- 十字キー 左　カメラ2ズームアウト
- △ボタン カメラ2リセット
- R1ボタンで加速度センサ/ジャイロセンサ/デプスセンサでマッピング

ROSのtopic
1. /quadcopter/camera_image  
カメラ2の画像:sensor_msgs/Image型
2. /quadcopter/quadcopter_pose  
ロボットの位置姿勢：geometry_msgs/Pose型
3. /quadcopter/output  
(Rogikeの名残：[参考](https://wrs2020.esa.io/posts/21))

## シンプルコントローラの単体でのコンパイル(試したいひと)
[コントローラのビルド](https://choreonoid.org/ja/manuals/latest/simulation/howto-build-controller.html)を参考にMakefileを作成した
```sh
cd ~/<catkin_ws>/src/quadcopter/sample/simplecontroller
make
make install
```
 適当にQuadcopterController_sample.cppの中身をいじればよい。もとは[サンプル](https://choreonoid.org/ja/manuals/latest/multicopter/index.html)のコピペ
