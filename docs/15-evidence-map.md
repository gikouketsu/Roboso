# 15. エビデンスマップ

主張と根拠を1対1で確認できるように整理します。非公開報告のページは事実確認にのみ使用し、個人情報を含む原文書はリポジトリに格納していません。

| 主張 | 公開コードまたは画像 | 非公開報告 | 判定 |
|---|---|---:|---|
| 4名チーム | - | p1 | 報告記録 |
| 本人はリーダー、機構・回路・プログラムを担当 | - | p1, p8-10 | 報告記録 |
| 発射機構は協働設計 | [`launcher-cad-and-prototype.jpg`](../media/mechanics/launcher-cad-and-prototype.jpg) | p8, p13 | 共同成果 |
| 発射機構を約11回改修 | - | p8 | 開発過程の記録 |
| ロボット全体のCADと試作機が存在 | [`robot-cad-and-prototype.jpg`](../media/overview/robot-cad-and-prototype.jpg) | p12 | 画像根拠 |
| 3輪を120°配置 | [`omni-wheel-geometry.jpg`](../media/chassis/omni-wheel-geometry.jpg) | p12-13 | 設計・実物記録 |
| 任意方向のオムニ運動学式 | コード実装なし | p12 | **設計説明のみ** |
| 画像取得は別スレッド | [`camera_spi4.py`](../python/spi/camera_spi4.py) | p16, p33 | コード確認 |
| フレーム共有はLockを使用 | [`camera_spi4.py`](../python/spi/camera_spi4.py) | p33 | コード確認 |
| queue / lock-free共有 | 該当実装なし | p10の記述とp33の付録が矛盾 | **実装済みとは扱わない** |
| HSVによる黄色検出 | [`camera_spi4.py`](../python/spi/camera_spi4.py), [実験画像](../media/experiments/vision-ball-detection.jpg) | p16, p33-34 | 検出処理は実装、画像の描画コードは未公開 |
| `distance_x/y` は画面偏差 | [`camera_spi4.py`](../python/spi/camera_spi4.py) | p16, p34 | ピクセル偏差、物理距離ではない |
| 偏差は `-125..125` に制限 | [`camera_spi4.py`](../python/spi/camera_spi4.py) | p34 | コード確認 |
| SPIは `x,y` の2 byte | Pythonと [`mbed/`](../mbed) | p23-24, p34-36 | コード確認 |
| STM32は `0xAA/0xBB` を準備 | [`mbed/`](../mbed) | p23-24 | コード確認、ホスト照合なし |
| checksum / 再送 | 該当実装なし | p10の記述と付録が不一致 | **未実装** |
| PWM過電流保護 | 電流検出コードなし | p10の記述のみ | **実装済みとは扱わない** |
| v1は閾値制御 | [`mbed_ball_tracking.cpp`](../mbed/mbed_ball_tracking.cpp) | p35-38 | コード確認 |
| v2はPID形式だが実質P制御 | [`mbed_ball_tracking_v2.cpp`](../mbed/mbed_ball_tracking_v2.cpp) | p38-42 | `Ki=Kd=0` |
| v3はI/D項と制限を追加 | [`mbed_ball_tracking_v3.cpp`](../mbed/mbed_ball_tracking_v3.cpp) | p42-46 | コードあり、**未テスト** |
| STM32拡張・ドライバPCBレイアウト | [`media/electronics`](../media/electronics) | p15 | スクリーンショットのみ |
| 元CAD / EDA / Gerber | リポジトリ内になし | 報告にも非添付 | **未提供** |
| 発射機構の基本動作 | 公開制御コードなし | p27-28 | 報告の定性記録 |
| 回収機構 | 公開制御コードなし | p28 | **未完了** |
| 投擲精度・長時間安定性 | 試験データなし | p28 | **未検証** |
| 発射距離モデル | 原始データなし | p17-20, p25 | 単位・数値に疑点、実測に使用不可 |
| 最優秀賞（2025年1月） | 従来のREADMEとリポジトリ所有者の情報 | 報告内に賞の記述なし | チーム成果として記載、一次資料なし |

## 公開していない根拠

- 学籍番号、本人の実名、他メンバーの氏名がある報告表紙
- 学科・学年・学籍情報があるタイトルページ
- 第三者の製品写真やコンセプト画像
- 計算が不整合な軌道図と、根拠のない「actual curve」
- 報告PDF全体
