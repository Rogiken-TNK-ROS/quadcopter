# quadcopter
開発の規約は以下に従う(2020/01/10)
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
 choreonoid QuadcopterJoystickWRS.cnoid
```

## シンプルコントローラの単体でのコンパイル(試したいひと)
[コントローラのビルド](https://choreonoid.org/ja/manuals/latest/simulation/howto-build-controller.html)を参考にMakefileを作成した
```sh
cd ~/<catkin_ws>/src/quadcopter/simplecontroller_sample
make
make install
```
 適当にQuadcopterController_sample.cppの中身をいじればよい。もとは[サンプル](https://choreonoid.org/ja/manuals/latest/multicopter/index.html)のコピペ
