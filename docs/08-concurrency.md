# 08. マルチスレッドとフレーム共有

## 実際の並行モデル

Python側は、カメラ取得用の `CameraCapture` スレッドと、画像処理・SPI送信を行うメインスレッドの2つで構成されます。

```mermaid
flowchart LR
    CAM[Camera device] -->|cap.read| PROD[CameraCapture thread]
    PROD -->|Lock: replace ret/frame| SLOT[(single latest-frame slot)]
    SLOT -->|Lock: read ret/frame| CONS[Main processing loop]
    CONS --> CV[HSV / contours]
    CV --> SPI[SPI transfer]
```

キューを使って全フレームを保存する構成ではありません。`self.frame` と `self.ret` の1組を `threading.Lock` で保護し、撮像側が最新値に置き換えます。したがって、報告の「queue」や「ロックフリー」という説明は、現在の公開コードの実装とは一致しません。

## この構成の性質

| 性質 | 効果 | 注意点 |
|---|---|---|
| 最新1フレームのみ | 古いフレームが蓄積しない | 中間フレームの欠落数を把握できない |
| 短いLock区間 | フレーム更新と取得の衝突を避ける | 取得後の画像処理はLock外 |
| ポーリング | 構成が単純 | フレームがない時も `continue` でビジーループになる |
| 明示的なバックプレッシャなし | 撮像を止めにくい | 処理能力より撮像が速い場合の挙動を計測していない |

## 停止処理

`KeyboardInterrupt` または `q` による終了後、メインスレッドが `camera_capture.stop()` と `spi.close()` を呼び、最後に `join()` します。一方、カメラ読み取りが失敗した場合、主実装の撮像スレッドは `self.ret` と `self.frame` を失敗状態に更新しません。そのため最後に成功した画像を再利用する可能性があります。

## 次の改善案（未実装）

- フレームIDと撮像タイムスタンプを共有する
- カメラ失敗時に `ret=False`, `frame=None` を明示的に公開する
- `threading.Event` で新規フレームを通知し、ビジーループを避ける
- 処理時間、フレーム年齢、欠落フレーム数をログする
- スレッド異常時に停止コマンドを送るフェイルセーフ経路を追加する

