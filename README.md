# Handheld-FFB-wheel  
# ハンドヘルド型FFB(フォースフィードバック)ハンドルコントローラー  

**●ここには詳細を置きます。概要はこちら↓に記述しました。**  
https://minkara.carview.co.jp/userid/3336538/blog/47191094/

**動作の状況の動画はこちら。**  
https://youtu.be/O-O_uvx_jfw 

**●初めに**  
　PCで車のゲームをプレイするとき、ゲームパッドではハンドル操作感が乏しいので運転が難しく、かといって机にFFBハンドルコントローラーを据え付けると使わないときに邪魔です。そこで気が向いた時だけ手軽に接続できるようにハンドヘルド型FFBハンドルコントローラーを作ってみました。  
![外観](https://github.com/Toshi2020/Handheld-FFB-wheel/assets/81674805/63f6caab-d3bb-4456-a868-326bf15af7f2)


**●主な部品**  
①DCモーター: タミヤ ソーラーモーター02 Amazon購入価格 721円  
②Arduino Pro Micro: AliExpress購入価格 568円  
③モータードライバー: BD6211F-E2 秋月購入価格 110円  
④ピッチ変換基板: P-05154 秋月購入価格 100円/9枚 単価11円  
⑤ショットキーバリアダイオード: 1S4 秋月購入価格 140円/10本 単価14円  
⑥ツェナーダイオード: Amazonでアソートキット 1200円/200本 単価6円  
⑦ボリューム: Bカーブ 手持ち品ですが秋月では60円  
⑧タクトスイッチ: 昔秋月で購入した手持ち品ですが今は 300円/10個 単価30円x3  
⑨分割ロングピンソケット: 秋月購入価格 80円  
⑩ジョイスティック: Amazon購入価格 799円/5個 単価160円  
その他は手持ちから流用ですが全部購入してもトータルで2,000円弱かと思います。(3Dプリンタのフィラメント代が別途必要です。)  
![パーツ](https://github.com/Toshi2020/Handheld-FFB-wheel/assets/81674805/86be3352-4e99-4de4-81db-a0ebfa543808)



**●ハードウェア**  
![回路図2](https://github.com/Toshi2020/Handheld-FFB-wheel/assets/81674805/7ae64294-45f1-4c68-b067-b7ecd0ff0fa1)


![メインボード](https://github.com/Toshi2020/Handheld-FFB-wheel/assets/81674805/5231a688-009f-42b3-93ba-2ab4186190f5)


・マイコンボードとしてArduino Pro Microを使います。PC用ゲームコントローラとして使うためにはArduino NANOやArduino Pro Miniではダメなので注意が必要です。今回AliexpressからUSB Type-Cコネクタの物を調達しました。  
・Arduinoは他に転用するかもしれないので秋月の分割ロングピンソケットを介して万能基板に取り付けて、ずれないように4隅をホットボンドで固定しています。またUSBコネクタ部分もホットボンドで補強しています。それとArduinoのピンは付属品ではなく秋月の細ピンヘッダ―を使っています。  
![メインボード2](https://github.com/Toshi2020/Handheld-FFB-wheel/assets/81674805/6a898933-bbb4-4940-89e8-db2107af40a8)


・USB給電だけで動かしたかったので、反力用のモーターとしては小電流で低回転のタミヤのソーラーモーター02を使います。このモーター、ネットで調べたのですが詳細スペックがわからず、おそらくAmazonで売られているRF-500TB-18280という型番の物ではないかと思います。タミヤで出している物は価格が少し高いのですがM2.5(?)の取り付けネジとピニオンギアが同梱されているのがメリットです。  
・モータードライバは秋月で購入したBD6211F-E2を同じく秋月のピッチ変更基板を使って組んでいます。  
・モーターを外部から回転させることになるので逆起電力により、モーターの回転と同方向に回すと供給電圧以上の電圧が発生し、逆方向に回すとストール電流以上の電流が流れるはずです(実測はしていないのですが)。マイコンやPCのUSB側に過電圧がかからないように1Aのショットキーダイオード1S4を介してモーターとドライバに給電しています。またドライバの定格電圧が7Vなので、保護のために5.6V 1Wのツェナーダイオードを入れています。電源ラインには100μの電解コンデンサを入れて電圧ドロップに対応したつもりです。  
・ボリュームは手持ちの関係でB型50kΩを使いましたがB型なら1k～100kΩの範囲で大丈夫だと思います。10kΩが良いかと思います。  
・ジョイスティックは、当初前後に倒してアクセル/ブレーキ、左右に倒してスイッチ(シフトアップ/シフトダウンに割り当て)としたのですが、操作に難があることがわかりY軸のみを使うことにしました。前後に倒したつもりでも左右に動いてしまうので、アクセル/ブレーキの微妙な調整が難しいのです。  
・パーツやケースの上下の固定はM3の6mmのビスを使用します。  
・モーターのシャフトには10歯のピニオンギアを使います。  
・モーターとボリュームは振動するのでネジロック剤を併用して取り付けています。  
・モーター、ボリューム、スイッチは万能基板とXHコネクタで接続しています。何度か抜き差ししているうちにコネクタの付け根が断線してしまったのでホットボンドで補強しています。  
![内部](https://github.com/Toshi2020/Handheld-FFB-wheel/assets/81674805/42c9838a-d1a7-43d4-8ef5-30850382eb8a)



**●ソフトウェア**  
・ArduinoでFFBホイールコントローラーを実現するためのソフトがGitHubにいくつかアップされているのですが、今回使用したもの以外ではAssettoCorsaでCPU使用率が99%を超えてシミュレーションが劇遅になってしまい使えませんでした。  
・ビルドするためにはこちらのAFFBWheel↓の、  
  https://github.com/vsulako/AFFBWheel  
  AFFBWheelフォルダから、  
  以下のファイル以外のファイルをMyAFFBWheelフォルダにコピーします。  
    AFFBWheel.ino, bb_i2c.cpp, bb_i2c.h, motor.cpp, motor.h, multiturn.cpp, multiturn.h   
・Arduino IDE 1.8.19以上でボードタイプはArduino Leonardoを選択してビルドします。  
・ビルドするとコントロールパネルのデバイスとプリンターにArduino Leonardoというゲームデバイスが出現します。  
・オリジナルのAFFBWheelではステア入力デバイスは光学エンコーダーを使うようになっていますが、ArduinoのA/Dに置き換えています。またオリジナルはEEPに調整用の値を書き込んで(設定用の汎用アプリがあるようです)細かい調整ができるようですが、今回は端折っています。  
・オリジナルを解析しつくしたわけではないので間違っているところもあるかもしれません。  
・オリジナルでは8軸のアナログと32個のボタン入力ができるのですが、今回はアナログ4軸と4ボタンのみ使っています。アナログはステア、アクセル、ブレーキ、クラッチです。  
・ステア入力はArduinoのA/Dが10bit分解能なので、値としては0～1023までしかとることができません。これでは1LSBの変化でステアがカクカク動いて使い物になりません。そこでA/Dの読みを100倍した値をフィルタに入れて見かけ上の分解能を上げることにしました。このフィルタは、ステアを動かさないときには重く、動かしているときには軽くすることで、安定性と応答性の両立を狙っています。  
・クラッチに関してはSW1を割り当てるという前提で、ボタンだけでなくアナログ軸にも出力を行うようにしています。AssettoCorsaはクラッチはアナログ軸にしか割り当てができないようなので。SW1を押した時にはスパッと切れて離すと0.5秒かけて戻るようにしています。  
・また、クラッチを踏んでいる間はブレーキを変化させないようにしています。これはヒールアンドトゥの操作を可能とするためです。(そんなことするか？はわかりませんけどね)  
・モーターに指示するPWM値は、ストール時に0.3[A]程度となるように最大値Fmaxを180としています。これはUSB2.0の最大電流が0.5[A]なので、それに対して逆起電力分を考慮した物です。  
・減速が1段のせいか、ステアから手を放した時のフォースによる戻り回転速度が速すぎるようで、ステアが行き過ぎて戻ってを繰り返すことがあったので、ステア回転速度に反比例してフォースを減少させるダンピング制御を入れました。  
・モーターには微小なPWMを与えても回転しない、いわゆるデッドゾーンがあります。デッドゾーンがあると、微小なフォースではステアが動かずに、フォースが徐々に上がった時にカクンと動き始めることになり、車の直進安定性が悪化します。そこでフォースをPWM出力する際にデッドゾーンを除去する処理を入れています。  
・通電時にステアをほぼ中央に戻す動きを行うようにしました。  
・SW1,3,4を押しながら通電すると耐久テストモードとなります。目標ステア角を徐々に増やしてMAXフォースで連続して左右に振ります。100サイクル回した限り問題ありませんでした。このとき非接触温度計でモーター温度45℃、ボード上のCPU温度50℃、電圧レギュレータ温度60℃でした。レギュレータが一番発熱するとは意外でした。またモーターが反転するときにボード上のLEDが一瞬暗くなるので電源ラインの電解コンデンサの容量はもっと増やした方が良いかと思います。  
・SW3,4を押しながら通電するとフォーステストモードとなります。ArduinoIDEのシリアルモニタでフォースとして-255～255の数値を送ると直接PWM出力します。この時デッドゾーン除去の処理も入ります。  

**●筐体について**  
・実はこのケースは3作目で、ステアの回転軸を30°手前に傾けています。前作は回転軸が横向きでした。昔ネジコンというコントローラーがあったのと、ラジコン用のプロポを意識したのですが、ボリュームの回転角300°を回すには横向きでは人体の構造上無理があることがわかりました。30°傾いていると肘の関節の回転を使えるので持ち替えずに回しきることができます。私はどちらも触ったことがないのですが、おそらくネジコンやプロポの回転範囲は狭いのだと思います。  
![平面図](https://github.com/Toshi2020/Handheld-FFB-wheel/assets/81674805/5e7b1c35-fd7a-4799-a871-7da1d2a4bf9b)



**●3Dデータ＆組付けについて**  
・使用するパーツによって修正が必要かもしれないので(特にタクトスイッチは昔に購入した物なので今の物とはサイズが違うかもです)STLだけでなくFusion360のファイルも入れておきます。"全パーツ.f3d"です。  
・スペーサーというモデルはさらに小型のボリュームを使う際に、シャフトが短い分を埋めるために使います。  
・ジョイスティックは、シャフトを±30°くらい傾けられるのですが、リニアな変化が得られるのは±22°くらいです。±30°倒せるようにするとリニアな領域が狭くてほとんどスイッチ的な使い方になってしまうので、ケースの上蓋の溝で±24°に制限をかけています。  
・ジョイスティックのノブは、スカート部分を切り取って使います。
![ジョイスティックノブ](https://github.com/Toshi2020/Handheld-FFB-wheel/assets/81674805/581d033a-f549-42dd-8869-932e57216218)


・スライサーはUltimaker-Curaを使っています。フィラメントはメインギア以外は汎用/PLAでStandard Quality - 0.2mmでインフィル密度20%、印刷は0.4mmノズルです。  
・サポートを外しやすいように、サポートオーバーハング角度は70°、サポート密度は5.0%としました。  
・メインギアだけはSuper Quality - 0.12mmでインフィル100%で印刷しました。  
・メインギアのマテリアルは最初PLAで印刷したのですがゴリゴリ感が気になり、もう少し柔軟性のあるPETGにしたところ改善が見られました。それではという事でさらに柔軟性のあるTPUで印刷したところさらに改善が見られ、さらに回転方向にバネ性を持たせる構造にすることでも改善が感じられました。メカ設計のプロから見たらとんでもないことかもしれませんが、私は素人なのでやれちゃいます。  
・メインギアのUltimaker-Curaのフィラメント設定は汎用/TPU 95Aです。  
・一応歯面にシリコングリスを薄く塗布してみましたが、効果は感じられませんでした。  
・手持ちのPETGフィラメントの線径が細いのだと思いますが、PLAと同じデータだとスカスカになってしまったので若干サイズを調整しています。  
・TPUギアって耐久的にどうかなと思ったので、ソフトで耐久試験ができるようにしました。  
・ギアの印刷で、1周に1か所スタート点ができるのですが、その部分の歯のサイズが少し大きくなり、ピニオンギアとのかみ合わせに滑らかさが欠けます。ボリュームの回転角が300°なので、ギアのスタート点にマーキングをして、その歯を避けて組み付ける必要があります。  
![ギア印刷](https://github.com/Toshi2020/Handheld-FFB-wheel/assets/81674805/c72e5f15-d06b-4da8-a91d-51f3a04e5927)


![ギアマーキング](https://github.com/Toshi2020/Handheld-FFB-wheel/assets/81674805/380f8ccc-00d2-444d-aaef-2218fa559cb8)


・印刷前にレベリングをちゃんと調整しておかないといけません。ノズルとベッドとの隙間が狭すぎるとケースのあわせ面のリブの溝の幅が狭まってしまって組めなくなるので注意です。  


