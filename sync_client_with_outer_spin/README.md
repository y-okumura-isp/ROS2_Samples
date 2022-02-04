# ROS2 sync client with outer spin

## はじめに

- タイムアウト付き同期待ち + 外側で spin する ros2 service の例。
- つまり、クライアントは外側の Executor で spin するが `spin_until_future_complete` で Future を待つ。
- 動作概要
  - クライアントでタイマーで定期的に起床して a, b を送り応答を最大 100 ms 待る。
  - サーバ側ではしばらく wait してから a+b を返します。
  - クライアントはタイマーでループしており、a を 1 ずつ加算する。  
    サーバは `(a * 10) % 200` [ms] wait するので wait 時間 [ms] は以下の様になる。  
    0 -> 10 -> 20 -> .. 90 -> 100 -> 110 -> ... -> 190 -> 0 -> 10 
    その為、 a < 10 までは良いのが a = 10-19 でタイムアウトし a=20 からまた動く様になる。

## Run & Result

```
# Build
# do `colcon build` and `source setup.bash`

# 1st console
ros2 run sync_client_with_outer_spin add_two_ints_server

# 2nd concole
ros2 run sync_client_with_outer_spin add_two_ints_client
```

client 側の実行結果例.
a=9 まではサーバ側の待ち時間が 90ms なので返信が返ります。
a=10-19 は 100-190 ms 待ちなので timeout します。
a=20 かはまた返る様になります。

```
[INFO] [1643938454.506649695] [add_two_ints_client]: start_wait a = 0
[INFO] [1643938454.507802607] [add_two_ints_client]: end_wait ret_value: 1
[INFO] [1643938455.505810109] [add_two_ints_client]: start_wait a = 1
[INFO] [1643938455.516803717] [add_two_ints_client]: end_wait ret_value: 3
[INFO] [1643938456.505850328] [add_two_ints_client]: start_wait a = 2
(snip)
[INFO] [1643938462.587122140] [add_two_ints_client]: end_wait ret_value: 17
[INFO] [1643938463.506057204] [add_two_ints_client]: start_wait a = 9
[INFO] [1643938463.597007145] [add_two_ints_client]: end_wait ret_value: 19
[INFO] [1643938464.506153307] [add_two_ints_client]: start_wait a = 10
[INFO] [1643938464.606490082] [add_two_ints_client]: end_wait timeout
(snip)
[INFO] [1643938473.506352555] [add_two_ints_client]: start_wait a = 19
[INFO] [1643938473.606706941] [add_two_ints_client]: end_wait timeout
[INFO] [1643938474.506412918] [add_two_ints_client]: start_wait a = 20
[INFO] [1643938474.507277208] [add_two_ints_client]: end_wait ret_value: 41
[INFO] [1643938475.506427060] [add_two_ints_client]: start_wait a = 21
[INFO] [1643938475.517497730] [add_two_ints_client]: end_wait ret_value: 43
[INFO] [1643938476.506476906] [add_two_ints_client]: start_wait a = 22
```

## 実装

- ClientNode でクライアント用の子ノードと子エグゼキュータを確保して子エグゼキュータで `spin_until_future_complete` して Future を待ちます。
- ClientNode 自体は上記の通り component で動かしているので「(外側の) spin 中に (別の Executor で) spin」して Future を待つことができます。
- ただし、本質的には 1 thread なので「Future の完了を待つ間、(外側の Executor の) 別の callback を処理する」ことはできません。

```
class ClientNode : public rclcpp::Node
{
public:
  explicit ClientNode(const rclcpp::NodeOptions & options)
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    client_node_ = rclcpp::Node::make_shared(std::string(this->get_name())  + "__client");
    executor_->add_node(client_node_);
    client_ = client_node_->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    // timer で何度もサービス呼び出し
    timer_ = create_wall_timer(1s, std::bind(&ClientNode::queue_async_request, this));
  }

  void queue_async_request()
  {
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    // request に値を詰めて送信
    auto future = client_->async_send_request(request);

    // 内部の executor で spin
    auto c =  executor_->spin_until_future_complete(future, 100ms);

    switch(c) {
      case rclcpp::FutureReturnCode::SUCCESS:
      // snip
    }
  }

private:
  rclcpp::Executor::SharedPtr executor_;
  rclcpp::Node::SharedPtr client_node_;
}
```

動作イメージ. best viewed with [mermaid-diagrams](https://chrome.google.com/webstore/detail/mermaid-diagrams/phfcghedmopjadpojhmmaffjmfiakfil)

``` mermaid
sequenceDiagram
   participant OuterExecutor
   participant ClientNode
   participant InnerExecutor
   participant Server

   activate OuterExecutor
   OuterExecutor->>ClientNode: timer callback
   deactivate OuterExecutor
   activate ClientNode
   ClientNode->>InnerExecutor: send_async & rcl_send_request
   ClientNode->>InnerExecutor: spin until future complete
   activate InnerExecutor
   deactivate ClientNode
   InnerExecutor->>Server: request
   Server->>InnerExecutor: response
   InnerExecutor->>InnerExecutor: spin & handle_response
   InnerExecutor->>InnerExecutor: promise & set_value(reqponse)
   InnerExecutor->>ClientNode: Future done
   deactivate InnerExecutor
   activate ClientNode
   ClientNode->>OuterExecutor: return
   deactivate ClientNode
   activate OuterExecutor
   OuterExecutor->>OuterExecutor: spin
   deactivate OuterExecutor
```

