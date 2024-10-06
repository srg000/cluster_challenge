import queue
import threading
import time
from enum import Enum
from swarmae.SwarmAEChannel import SwarmAEChannel

channel = SwarmAEChannel(
    topic=3, channel_name="channel_1", is_keepalived=True, lifetime=3600, is_available=True
)


class MessageType(Enum):
    """
    MessageType枚举类，定义了消息的类型：
    - BROADCAST: 广播消息，发送给所有接收者
    - DIRECTED: 定向消息，发送给具有特定ID的接收者
    """
    BROADCAST = 1
    DIRECTED = 2
    TOPIC = 3


class Message:
    def __init__(self, sender, receiver, message_type, content):
        """
        Message对象
        Args:
            sender (int): 消息发送者id
            receiver (int): 消息接收者id
            message_type (int): 消息类型
            content (str): 消息内容
        Returns:
            None
        """
        self.sender = sender
        self.receiver = receiver
        self.type = message_type
        self.content = content


class MessageSystem:
    def __init__(self, channel):
        """
        message_queue (queue.Queue): 消息队列，用于存储待处理的消息。
        stop_event (threading.Event): 停止事件，用于控制处理线程何时停止。
        processing_threads (list): 处理线程列表，存储所有正在运行的处理线程。
        confirmations (dict): 消息确认字典，存储每个消息的唯一标识符和对应的确认状态。
        retries (dict): 消息重试字典，存储每个消息的唯一标识符和对应的重试次数。
        """
        self.message_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.processing_threads = []
        self.confirmations = {}  # 用于存储消息确认的字典
        self.retries = {}  # 用于存储重试次数的字典
        self.channel = channel

    def start(self, num_workers=4):
        """
        启动消息处理线程
        Args:
            num_workers (int, optional): 线程数量，默认为4。
        """
        for _ in range(num_workers):
            t = threading.Thread(target=self.process_messages)
            t.daemon = True
            self.processing_threads.append(t)
            t.start()

    def stop(self):
        self.stop_event.set()
        for t in self.processing_threads:
            t.join()

    def process_messages(self):
        # 当停止事件未设置时，继续处理消息
        while not self.stop_event.is_set():
            try:
                # 从消息队列中获取消息，阻塞模式，超时时间为1秒
                message = self.message_queue.get(block=True, timeout=1)
                # 处理消息
                self.handle_message(message)
                # 标记消息队列中的任务已完成，以便后续垃圾回收
                self.message_queue.task_done()
            except queue.Empty:
                # 捕获队列为空的异常，此时表示超时，继续检查
                pass  # Timeout, continue checking

    def handle_message(self, message):
        # 根据接收者和消息类型处理消息
        print(f"Processing message: {message}")
        # 假设有一个分发函数，根据接收者ID将消息发送给相应的无人车或无人机
        self.dispatch_message(message)
        # 处理确认和重试逻辑
        self.handle_confirmation_and_retry(message)

    def dispatch_message(self, message):
        # 将Message对象中的字符串内容转换为字节流
        msg_data = message.content.encode('utf-8')  # 使用UTF-8编码将字符串转换为字节流

        # 调用SDK的send_message方法来发送消息
        try:
            code, frame_timestamp = self.channel.send_message(
                message.sender,  # 发送方节点ID
                msg_data,  # 转换后的字节流数据
                int(time.time() * 1000),  # 使用当前时间作为帧时间戳
                MessageType.TOPIC  # 会话主题
            )

            # 处理发送结果
            if code == 200:
                print(f"Message dispatched successfully with code {code} and timestamp {frame_timestamp}")
            else:
                print(f"Error dispatching message: code {code}")

        except Exception as e:
            print(f"An exception occurred while dispatching the message: {e}")

    def handle_confirmation_and_retry(self, message):
        # 处理消息确认和重试的逻辑
        message_id = (message.sender, message.receiver, message.type)
        if message_id not in self.confirmations:
            self.confirmations[message_id] = False
            self.retries[message_id] = 0

        # 重试逻辑（如果未收到确认且重试次数未达到上限）
        if not self.confirmations[message_id] and self.retries[message_id] < 3:
            self.retries[message_id] += 1
            self.message_queue.put(message)  # 重新放入队列进行重试
        else:
            # 清除确认和重试记录（如果收到确认或达到重试上限）
            self.confirmations.pop(message_id, None)
            self.retries.pop(message_id, None)

    def send_message(self, sender, receiver, message_type, content):
        message = Message(sender, receiver, message_type, content)
        self.message_queue.put(message)

    def confirm_message(self, sender, receiver, message_type):
        message_id = (sender, receiver, message_type)
        if message_id in self.confirmations:
            self.confirmations[message_id] = True


# 在主代码中集成 MessageSystem

message_system = MessageSystem(channel)  # 创建消息系统实例
message_system.start()  # 启动消息处理线程

# 示例：发送消息(目前只用广播方式传递信息)
sender = "vehicle_1"
receiver = None
# receiver = 'uav_1'
message_type = MessageType.BROADCAST
content = "Take off"
message_system.send_message(sender, receiver, message_type, content)

# ... [在适当的位置处理消息确认和接收逻辑] ...
# 例如，在无人机执行命令后，调用 confirm_message 方法来确认消息已处理
message_system.confirm_message(sender, receiver, message_type)

# 在程序结束时停止消息系统
message_system.stop()
