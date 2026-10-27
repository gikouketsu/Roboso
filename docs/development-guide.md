# 開発・再検証ガイド

## 読むための環境と、動かすための環境

| 作業 | 必要な環境 | 今回確認できる範囲 |
|---|---|---|
| 資料とソースの確認 | GitHubまたはローカルエディタ | README、モジュール入口、画像と根拠 |
| リポジトリ検査 | Python 3.10以上、標準ライブラリ | Markdownのローカルリンク、Python構文 |
| 偏差byteの確認 | Python 3.10以上、標準ライブラリ | 符号と飽和の変換、単体テスト |
| 画像・SPI実行 | Raspberry Pi/Linux、依存パッケージ、カメラと接続機器 | 当時のコードあり、今回の実機再検証なし |
| MCUビルド | 対応Mbed工程とツールチェーン | 完全工程未提供、ビルド未検証 |
| 機構再製作 | 元CAD、製造条件、部品と実機 | 画像のみ、原データ未提供 |

## ハードウェア不要の確認

リポジトリのルートで実行します。外部パッケージをインストールする必要はありません。

```sh
python tools/check_repository.py
python -m unittest discover -s tests -v
python python/tools/inspect_packet.py encode -51 -39
python python/tools/inspect_packet.py decode 205 217
```

`check_repository.py` は既存のカメラ/SPIコードをimportせず、ASTによる構文確認とローカルMarkdownリンクの存在確認だけを行います。実機起動、C++ビルド、外部URLのアクセス、Mermaid描画は行いません。単体テストは新規のbyte確認ツールだけを対象にしており、ロボットの動作を認定する試験ではありません。

## 実機を再立ち上げする場合

1. [部品・原資料の棚卸し](../electronics/component-inventory.md)で、不足している版・型式・設計データを確認する。
2. [Mbed](../mbed/README.md)で対象版を選び、対応するビルド環境を構成する。
3. [信号一覧](../electronics/interfaces.md)と実配線を照合する。
4. [統合手順](../hardware/integration.md)に沿って、ロジック・通信・個別駆動から確認する。
5. [試験計画](10-verification.md)に条件、方法、試行数と生データを追加する。

当時のソースは実験履歴として保持しています。コード修正・配線変更・制御パラメータ変更を行う場合は、commitと試験記録を対応させてください。

## 現在の追加コードの位置付け

`python/tools/inspect_packet.py`、`tools/check_repository.py`、`tests/` とCIは、今回の作品集整備時に新規追加した開発補助です。当時の実機に搭載されていた機能や、過去の試験成果としては扱いません。原ソースとこの補助コードを混同しないよう、各ディレクトリのREADMEから参照しています。
