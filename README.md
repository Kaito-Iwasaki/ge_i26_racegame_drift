# racegame

鈴鹿サーキットと秋名の下りを走る、3D レースゲームです。車は 1 台です。ドリフトを実装済みです（詳細は `README_DRIFT.md`）。

コースの線形は OpenStreetMap（relation 284570、Suzuka International Racing Course）、高さは国土地理院の標高タイルです。

- 地図データ: © OpenStreetMap contributors
- 標高: 出典 国土地理院

画像は `assets/graphics/`、音声は `assets/sound/` に置きます。コースデータは `assets/track/`（`suzuka.txt` と `akina.txt`）です。引数なしで起動すると秋名、`./racegame suzuka` で鈴鹿を選べます。

## 開き方

Cursor または VS Code でこのフォルダを開き、**Reopen in Container** を選びます。初回はイメージのビルドに数分かかります。

## 操作

キーボードとゲームパッドは、一般的な 3D レースゲームと同じです。ハンドブレーキ（ドリフト用）があります。

| 操作 | キーボード | ゲームパッド |
|---|---|---|
| アクセル | W / ↑ | RT（R2）。十字キー上でも可 |
| ブレーキ / 後退 | S / ↓ | LT（L2）。十字キー下でも可 |
| ハンドブレーキ（ドリフト） | Space | A / RB |
| ステアリング | A D / ← → | 左スティック、十字キー左右 |
| リスタート | R | Back（View / Share） |
| カメラ | C | Y（△） |
| ポーズ | P | Start（Menu / Options） |
| 操作表示 | H | |

画面は追従、近め、遠めの 3 つです。ギアはオートマです。HUD 右上に FPS も表示します。

## ドリフト

`Car::stepPhysics`（`src/car.cpp`）で速度を2次元ベクトルとして扱い、横滑りとドリフト状態（入り／維持／終わり）を実装しています。入り方はハンドブレーキ、パワースライド、ブレーキングドリフト、ステアリングのフリック。タイヤの煙、録音ベースのエンジン／ドリフト音、秋名の下りコースも入っています。詳細は `README_DRIFT.md`、検証は `src/` のテストハーネス（`drift_test.cpp` など）です。

## ブラウザ版

```bash
emcmake cmake -B build
cmake --build build
python3 -m http.server 8080
```

`http://localhost:8080/build/racegame.html`

## Linux 版

```bash
cmake -B build-native
cmake --build build-native
./build-native/racegame
```

画面は `http://localhost:6080`（パスワード `vscode`）です。コンテナ内では音は鳴りません。

ロジックだけの確認:

```bash
./build-native/racegame --self-test
```
