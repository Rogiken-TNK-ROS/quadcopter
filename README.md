# quadcopter

## quadcopterリポジトリ
```sh
cd ~/<catkin_ws>/src
git clone https://gitlab.com/rogiken-tnk-ros/quadcopter.git
cp choreonoid/src/MulticopterPlugin/exportdecl.h ../devel/include/choreonoid-1.8/cnoid/src/MulticopterPlugin
```

## シンプルコントローラのコンパイル
[コントローラのビルド](https://choreonoid.org/ja/manuals/latest/simulation/howto-build-controller.html)を参考にMakefileを作成した
```sh
cd ~/<catkin_ws>/src/quadcopter/QuadcopterControllerWRS
make
make install
```
 適当にQuadcopterControllerWRS.cppの中身をいじればよい。もとは[サンプル](https://choreonoid.org/ja/manuals/latest/multicopter/index.html)のコピペ

## プロジェクトの実行
```sh
cd ~/<catkin_ws>/src/quadcopter/cnoid
 choreonoid QuadcopterJoystick.cnoid
```