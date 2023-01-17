# atomfly_kanazawa

AtomFlyの飛行制御プログラムを作っていきます。

[pico_copter](https://github.com/kouhei1970/pico_copter)を一部流用します。

### ビルド環境

#### Vscode

##### Ubuntu

1. [こちら](https://code.visualstudio.com/Download)から.debパッケージをダウンロード
2. ダウンロードしたフォルダにて
```
dpkg -i debファイル名
```

##### Mac
```
brew install --cask visual-studio-code
```

##### Win
[こちら](https://code.visualstudio.com/Download)からインストーラーをダウンロードして実行


#### PlattofomIO
1. [公式ページ](https://platformio.org/)https://platformio.org/
2. [参考](https://qiita.com/JotaroS/items/1930f156aab953194c9a)https://qiita.com/JotaroS/items/1930f156aab953194c9a

#### git
##### Ubuntu
たぶんインストール済み
```
apt install git
```

##### Mac
```
brew install git
```

##### Win

[こちら](https://gitforwindows.org/)からインストーラをダウンロードしてください

### ビルド
適当なディレクトリにて
```
git clone https://github.com/kouhei1970/atomfly_kanazawa
```

1. vscode（PltformIO）を開いて、atomfly_kanazawaプロジェクトを開く
2. 下部のCheck markのアイコンをクロックしてビルドを実行

### 操縦コントローラ

[Controller code](https://github.com/kouhei1970/m5stick_joystick)

![Stick map](new_controller.png)
