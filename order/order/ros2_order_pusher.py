#!/usr/bin/env python3
import time
import yaml
import requests
from typing import Optional, Dict, Any, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 你的 srv（先放到 tm_robot_if/srv 並 colcon build）
# GetNextOrder.srv
#   string worker_id
#   ---
#   bool   success
#   string order_id
#   string order_yaml
#   string error
#
# CompleteOrder.srv
#   string order_id
#   string status   # "success" or "failed"
#   ---
#   bool   success
#   string error
from tm_robot_if.srv import GetNextOrder, CompleteOrder

DEFAULT_BASE_URL = "http://localhost:8001"

class OrderGatewaySimple(Node):
    """
    訂單閘道（簡化版，單執行緒）
      - /get_next_order：從後端拉下一張訂單（可設 long-poll）
      - /complete_order：回報訂單完成/失敗
      -（可選）自動推播到 /hospital/new_order（完整 YAML）
    不使用 QoSProfile / ReentrantCallbackGroup / MultiThreadedExecutor
    """

    def __init__(self):
        super().__init__('order_gateway_simple')

        self.base_url = self.declare_parameter('base_url', DEFAULT_BASE_URL).get_parameter_value().string_value
        self.enable_push = self.declare_parameter('enable_push', True).get_parameter_value().bool_value
        self.poll_interval = self.declare_parameter('poll_interval', 3.0).get_parameter_value().double_value
        self.long_poll_seconds = self.declare_parameter('long_poll_seconds', 0.0).get_parameter_value().double_value
        self.long_poll_step = self.declare_parameter('long_poll_step', 1.0).get_parameter_value().double_value
        self.http_timeout_next = self.declare_parameter('http_timeout_next', 5.0).get_parameter_value().double_value
        self.http_timeout_complete = self.declare_parameter('http_timeout_complete', 5.0).get_parameter_value().double_value

        self.order_pub = self.create_publisher(String, '/hospital/new_order', 10)

        # Services
        self.srv_next = self.create_service(GetNextOrder, 'get_next_order', self.get_next_order_cb)
        self.srv_complete = self.create_service(CompleteOrder, 'complete_order', self.complete_order_cb)

        self.timer = None
        if self.enable_push:
            self.timer = self.create_timer(self.poll_interval, self._poll_and_push)

        # 去重控制（本次程式運行內）
        self.last_pushed_order_id: Optional[str] = None
        self.inflight = False  # 避免 timer 重入

        self.get_logger().info(
            f"[OrderGatewaySimple] base_url={self.base_url} | push={self.enable_push} | poll={self.poll_interval}s | long_poll={self.long_poll_seconds}s"
        )

    # ===== Service: GetNextOrder =====
    def get_next_order_cb(self, req: GetNextOrder.Request, res: GetNextOrder.Response):
        """
        取下一張訂單。
        注意：此節點為單執行緒，若 long_poll_seconds>0，這個 callback 會阻塞直到有單或超時。
        """
        try:
            if self.long_poll_seconds > 0:
                deadline = time.time() + self.long_poll_seconds
                while True:
                    ok, order, order_yaml, err = self._http_fetch_next()
                    if ok and order:
                        res.success = True
                        res.order_id = str(order.get('order_id', ''))
                        res.order_yaml = order_yaml
                        res.error = ""
                        self.get_logger().info(f"[get_next_order] order_id={res.order_id}")
                        return res
                    if time.time() >= deadline:
                        res.success = False
                        res.order_id = ""
                        res.order_yaml = ""
                        res.error = "no order"
                        return res
                    time.sleep(max(0.0, self.long_poll_step))
            else:
                ok, order, order_yaml, err = self._http_fetch_next()
                if ok and order:
                    res.success = True
                    res.order_id = str(order.get('order_id', ''))
                    res.order_yaml = order_yaml
                    res.error = ""
                else:
                    res.success = False
                    res.order_id = ""
                    res.order_yaml = ""
                    res.error = err or "no order"
                return res

        except Exception as e:
            res.success = False
            res.order_id = ""
            res.order_yaml = ""
            res.error = str(e)
            self.get_logger().exception("get_next_order exception")
            return res

    # ===== Service: CompleteOrder =====
    def complete_order_cb(self, req: CompleteOrder.Request, res: CompleteOrder.Response):
        """回報訂單完成（或失敗）"""
        try:
            status = (req.status or "success").lower()
            if status not in ("success", "failed"):
                status = "success"

            payload = {"order_id": req.order_id, "status": status}
            r = requests.post(
                f"{self.base_url}/api/ros2/order/complete",
                json=payload,
                timeout=self.http_timeout_complete
            )
            ok = (r.status_code == 200)
            res.success = ok
            res.error = "" if ok else f"http {r.status_code}: {r.text}"

            if ok:
                self.get_logger().info(f"✅ complete_order ok order_id={req.order_id} status={status}")
                # 保險：結單後清掉記憶，下一輪推播可拿到新單
                self.last_pushed_order_id = None
            else:
                self.get_logger().error(f"❌ complete_order fail {res.error}")
            return res

        except Exception as e:
            res.success = False
            res.error = str(e)
            self.get_logger().exception("complete_order exception")
            return res

    # ===== Timer: Poll & Push（可關閉）=====
    def _poll_and_push(self):
        if self.inflight:
            return
        self.inflight = True
        try:
            ok, order, order_yaml, err = self._http_fetch_next()
            if not (ok and order):
                return

            order_id = str(order.get('order_id', ''))
            if not order_id:
                self.get_logger().warn("取到訂單但缺少 order_id，略過")
                return

            # 單次運行去重：相同 order_id 不重發
            if order_id == self.last_pushed_order_id:
                return

            self.order_pub.publish(String(data=order_yaml))
            self.last_pushed_order_id = order_id
            self.get_logger().info(f"🆕 推送訂單到 /hospital/new_order → {order_id}")

        except Exception as e:
            self.get_logger().error(f"推播失敗：{e}")
        finally:
            self.inflight = False

    # ===== HTTP helpers =====
    def _http_fetch_next(self) -> Tuple[bool, Optional[Dict[str, Any]], str, str]:
        """
        呼叫後端 /api/ros2/order/next
        回傳: (ok, order_dict|None, order_yaml, error_msg)
        """
        # 可選：先看看系統狀態（不中斷）
        try:
            s = requests.get(f"{self.base_url}/api/system/status", timeout=3)
            if s.status_code != 200:
                self.get_logger().warn("system/status 不可用")
        except Exception:
            self.get_logger().warn("檢查 system/status 失敗")

        try:
            r = requests.get(f"{self.base_url}/api/ros2/order/next", timeout=self.http_timeout_next)
        except Exception as e:
            return False, None, "", f"http error: {e}"

        if r.status_code == 204:
            return True, None, "", "no order"
        if r.status_code != 200:
            return False, None, "", f"http {r.status_code}: {r.text}"

        try:
            data = r.json() or {}
            order = data.get("order", {}) or {}
            order_yaml = data.get("yaml") or yaml.safe_dump(order, allow_unicode=True)
            return True, order, order_yaml, ""
        except Exception as e:
            return False, None, "", f"parse error: {e}"


def main():
    rclpy.init()
    node = OrderGatewaySimple()
    try:
        # 單執行緒 spin（不使用 MultiThreadedExecutor）
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
